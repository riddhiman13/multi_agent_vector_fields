import os
import roslaunch
import rospy
from rospkg import RosPack
import yaml

def main():
    rospy.init_node('multi_agent_feedback_node_launcher', anonymous=True)

    # Retrieve all private parameters and pass them as arguments
    private_params = rospy.get_param_names()
    args = {}
    for param in private_params:
        if param.startswith(rospy.get_name() + '/'):  # Check for parameters in the node's namespace
            param_name = param.split('/')[-1]         # Extract the parameter name
            args[param_name] = rospy.get_param(param)
    rospy.loginfo(f"[multi_agent_feedback_node_launcher] Private parameters: {args}")

    # Check for required parameters in args
    try:
        param_env_scene = args['env_scene']
    except KeyError as e:
        rospy.logerr(f"Parameter '~{e.args[0]}' is required but not set.")
        raise rospy.ROSInitException(f"Missing required parameter: '~{e.args[0]}'")

    # Use rospkg to get the path of the scene file
    rospack = RosPack()
    try:
        multi_agent_vector_fields_path = rospack.get_path('multi_agent_vector_fields')
    except Exception as e:
        rospy.logerr(f"Failed to find package 'multi_agent_vector_fields': {e}")
        raise rospy.ROSInitException("Could not locate 'multi_agent_vector_fields' package.")
    
    scene_file_path = f"{multi_agent_vector_fields_path}/scene/{param_env_scene}"
    if not os.path.isfile(scene_file_path):
        rospy.logwarn(f"[multi_agent_feedback_node_launcher] Scene file not found: {scene_file_path}")
        param_file = f"{multi_agent_vector_fields_path}/config/multi_agent_parameter.yaml"
    else:
        param_file = scene_file_path

    # Load the parameter file
    try:
        with open(param_file, 'r') as file:
            rospy.loginfo(f"[multi_agent_feedback_node_launcher] Loaded parameters from: {param_file}")
            params = yaml.safe_load(file)
            for key, value in params.items():
                rospy.set_param(f"/{key}", value)
                rospy.loginfo(f"[multi_agent_feedback_node_launcher] Set parameter '/{key}' as '{value}'")
    except Exception as e:
        rospy.logerr(f"Failed to load parameter file '{param_file}': {e}")
        raise rospy.ROSInitException(f"Error loading parameter file: {param_file}")

    # Launch the multi_agent_feedback_node using ROSLaunchParent
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    # Define the launch file and arguments
    package = "multi_agent_vector_fields"
    launch_file = "multi_agent_feedback_node.launch"
    launch_file_path = roslaunch.rlutil.resolve_launch_arguments([package, launch_file])[0]
    launch_args = [f"env_scene:={param_env_scene}"]

    rospy.loginfo(f"[multi_agent_feedback_node_launcher] Launching file: {launch_file_path} with args: {launch_args}")

    # Start the launch file
    parent = roslaunch.parent.ROSLaunchParent(uuid, [(launch_file_path, launch_args)])
    parent.start()

    rospy.loginfo("[multi_agent_feedback_node_launcher] Launching multi_agent_feedback_node...")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("[multi_agent_feedback_node_launcher] Shutting down...")
        parent.shutdown()

if __name__ == "__main__":
    main()
