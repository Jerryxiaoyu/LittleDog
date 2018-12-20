import pydrake
from pydrake.all import (
    AddModelInstanceFromUrdfStringSearchingInRosPackages,
    ConstantVectorSource,
    Context,
    DiagramBuilder,
    FloatingBaseType,
    LeafSystem,
    PiecewisePolynomial,
    PortDataType,
    RigidBodyFrame,
    RigidBodyPlant,
    RigidBodyTree,
    Shape,
    SignalLogger,
    Simulator,
)

import math
import meshcat
import meshcat.transformations as tf


class MeshcatRigidBodyVisualizer(LeafSystem):
    def __init__(self,
                 rbtree,
                 draw_timestep=0.033333,
                 prefix="RBViz",
                 zmq_url="tcp://127.0.0.1:6000"):  #127.0.0.1
        LeafSystem.__init__(self)
        self.set_name('meshcat_visualization')
        self.timestep = draw_timestep
        self._DeclarePeriodicPublish(draw_timestep, 0.0)
        self.rbtree = rbtree

        self._DeclareInputPort(PortDataType.kVectorValued,
                               self.rbtree.get_num_positions() +
                               self.rbtree.get_num_velocities())

        # Set up meshcat
        self.prefix = prefix
        self.vis = meshcat.Visualizer(zmq_url=zmq_url)
        self.vis[self.prefix].delete()

        # Publish the tree geometry to get the visualizer started
        self.PublishAllGeometry()

    def PublishAllGeometry(self):
        n_bodies = self.rbtree.get_num_bodies()-1
        all_meshcat_geometry = {}
        for body_i in range(n_bodies):

            body = self.rbtree.get_body(body_i+1)
            # TODO(gizatt) Replace these body-unique indices
            # with more readable body.get_model_name() or other
            # model index information when an appropriate
            # function gets bound in pydrake.
            body_name = body.get_name() + ("(%d)" % body_i)

            visual_elements = body.get_visual_elements()
            this_body_patches = []
            for element_i, element in enumerate(visual_elements):
                element_local_tf = element.getLocalTransform()
                if element.hasGeometry():
                    geom = element.getGeometry()

                    geom_type = geom.getShape()
                    if geom_type == Shape.SPHERE:
                        meshcat_geom = meshcat.geometry.Sphere(geom.radius)
                    elif geom_type == Shape.BOX:
                        meshcat_geom = meshcat.geometry.Box(geom.size)
                    elif geom_type == Shape.CYLINDER:
                        meshcat_geom = meshcat.geometry.Cylinder(
                            geom.length, geom.radius)
                        # In Drake, cylinders are along +z
                        # In meshcat, cylinders are along +y
                        # Rotate to fix this misalignment
                        extra_rotation = tf.rotation_matrix(
                            math.pi/2., [1, 0, 0])
                        print extra_rotation
                        element_local_tf[0:3, 0:3] = \
                            element_local_tf[0:3, 0:3].dot(
                                extra_rotation[0:3, 0:3])
                    elif geom_type == Shape.MESH:
                        meshcat_geom = \
                            meshcat.geometry.ObjMeshGeometry.from_file(
                                geom.resolved_filename[0:-3] + "obj")
                        # respect mesh scale
                        element_local_tf[0:3, 0:3] *= geom.scale
                    else:
                        print "UNSUPPORTED GEOMETRY TYPE ",\
                              geom.getShape(), " IGNORED"
                        continue

                    def rgba2hex(rgb):
                        ''' Turn a list of R,G,B elements (any indexable
                        list of >= 3 elements will work), where each element
                        is specified on range [0., 1.], into the equivalent
                        24-bit value 0xRRGGBB. '''
                        val = 0.
                        for i in range(3):
                            val += (256**(2 - i)) * (255. * rgb[i])
                        return val
                    self.vis[self.prefix][body_name][str(element_i)]\
                        .set_object(meshcat_geom,
                                    meshcat.geometry.MeshLambertMaterial(
                                        color=rgba2hex(element.getMaterial())))
                    self.vis[self.prefix][body_name][str(element_i)].\
                        set_transform(element_local_tf)

    def _DoPublish(self, context, event):
        self.draw(context)

    def draw(self, context):
        ''' Evaluates the robot state and draws it.
            Can be passed either a raw state vector, or
            an input context.'''
        if isinstance(context, Context):
            positions = self.EvalVectorInput(context, 0).get_value()[0:self.rbtree.get_num_positions()]  # noqa
        else:
            positions = context[0:self.rbtree.get_num_positions()]

        kinsol = self.rbtree.doKinematics(positions)

        body_fill_index = 0
        for body_i in range(self.rbtree.get_num_bodies()-1):
            tf = self.rbtree.relativeTransform(kinsol, 0, body_i+1)
            body_name = self.rbtree.get_body(body_i+1).get_name() \
                + ("(%d)" % body_i)
            self.vis[self.prefix][body_name].set_transform(tf)

    def animate(self, log, resample=True):
        # log - a reference to a pydrake.systems.primitives.SignalLogger that
        # contains the plant state after running a simulation.
        # resample -- should we do a resampling operation to make
        # the samples more consistent in time? This can be disabled
        # if you know the draw_timestep passed into the constructor exactly
        # matches the sample timestep of the log.

        if type(log) is SignalLogger:
            t = log.sample_times()
            x = log.data()

            if resample:
                import scipy.interpolate

                t_resample = np.arange(0, t[-1], self.timestep)
                x = scipy.interpolate.interp1d(t, x, kind='linear', axis=1)(t_resample)  # noqa
                t = t_resample

        # TODO(russt): Replace PiecewisePolynomial with Trajectory if I ever
        # add the pydrake bindings for the base class.
        elif type(log) is PiecewisePolynomial:
            t = np.arange(log.start_time(), log.end_time(), self.timestep)
            x = np.hstack([log.value(dt) for dt in t])

        def animate_update(i):
            self.draw(x[:, i])

        for i in range(t.shape[0]):
            animate_update(i)
            time.sleep(self.timestep)