/* TASK SEQUENCER - Contains all recepies for grasping, and other tasks
Authors: George Jose Pollayil - Mathew Jose Pollayil - Stefano Angeli
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com, stefano.angeli@ing.unipi.it  */

#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"

#include "abb_wrapper_control/TaskSequencer.h"
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Dense>
#include <moveit/move_group_interface/move_group_interface.h>

TaskSequencer::TaskSequencer(ros::NodeHandle& nh_){
    
    // Initializing Node Handle
    this->nh = nh_;

    // Parsing the task params
    if(!this->parse_task_params()){
        ROS_ERROR("The parsing of task parameters went wrong. Be careful, using default values...");
    }

    // Initializing Abb Client (TODO: Return error if initialize returns false)
    this->abb_client.initialize(this->nh);

    // Moveit names

	if(!nh_.getParam("/abb/group_name", group_name)){
        ROS_ERROR("Failed to load the move group name!");
    };

    ROS_INFO("The move group name is: %s", group_name.c_str());
 
    //End-effector name use for planning
    
	if(!nh_.getParam("/abb/end_effector_name", end_effector_name)){
        ROS_ERROR("Failed to load the end_effector_name!");
    };

    ROS_INFO("The end-effector name is: %s", end_effector_name.c_str());

    // Initializing other moveit stuff (robot model, kinematic model and state)
    this->robot_model_loader_ptr.reset(new robot_model_loader::RobotModelLoader("robot_description"));
    this->kinematic_model = this->robot_model_loader_ptr->getModel();
    ROS_INFO("Model frame: %s", this->kinematic_model->getModelFrame().c_str());
    this->kinematic_state.reset(new robot_state::RobotState(this->kinematic_model));
    
    // Get the array of active joints
    this->number_of_active_joints = this->kinematic_model->getActiveJointModels();

    // Setting the task service names
    this->example_task_service_name = "example_task_service";
    this->template_task_service_name = "template_task_service";

    // Advertising the services
    this->example_task_server = this->nh.advertiseService(this->example_task_service_name, &TaskSequencer::call_example_task, this);
    this->template_task_server = this->nh.advertiseService(this->template_task_service_name, &TaskSequencer::call_template_task, this);

    // Initializing other control values 
    this->waiting_time = ros::Duration(20.0);
    this->null_joints.resize(this->number_of_active_joints.size());
    std::fill(this->null_joints.begin(), this->null_joints.end(), 0.0);

    //
    this->gripper_close_open.data = true;

    this->dice_sub=this->nh.subscribe("/dice_pose",100,&TaskSequencer::call_pose,this);    
    // Spinning once
    ros::spinOnce();
}


void TaskSequencer::call_pose(const geometry_msgs::PoseStamped& pose)
{
    this->dice_pose = pose;
    //ROS_INFO("Dice: %f,%f,%f",  this->dice_pose.pose.position.x,this->dice_pose.pose.position.y,this->dice_pose.pose.position.z);

}
TaskSequencer::~TaskSequencer(){
    std::cout << "Destructor executed" << std::endl;
    // Nothing to do here yet
}

// Parameters parsing

bool TaskSequencer::parse_task_params(){

    bool success = true;

    if(!ros::param::get("/task_sequencer/grasp_transform", this->grasp_transform)){
		ROS_WARN("The param 'grasp_transform' not found in param server! Using default.");
		this->grasp_transform.resize(6);
        std::fill(this->grasp_transform.begin(), this->grasp_transform.end(), 0.0);
		success = false;
	}

    // Converting the grasp_transform vector to geometry_msgs Pose
    this->grasp_T = this->convert_vector_to_pose(this->grasp_transform);

    if(!ros::param::get("/task_sequencer/pre_grasp_transform", this->pre_grasp_transform)){
		ROS_WARN("The param 'pre_grasp_transform' not found in param server! Using default.");
		this->pre_grasp_transform.resize(6);
        std::fill(this->pre_grasp_transform.begin(), this->pre_grasp_transform.end(), 0.0);
		success = false;
	}

    // Converting the pre_grasp_transform vector to geometry_msgs Pose
    this->pre_grasp_T = this->convert_vector_to_pose(this->pre_grasp_transform);
    
    //
    if(!ros::param::get("/task_sequencer/home_joints", this->home_joints)){
		ROS_WARN("The param 'home_joints' not found in param server! Using default.");
		this->home_joints = {0.0,-0.0364, 0.0, 0.5051, 0.0, 1.1522, 0.0};
		success = false;
	}

    if(!ros::param::get("/task_sequencer/joint_position_A", this->joint_pos_A)){
		ROS_WARN("The param 'joint_position_A' not found in param server! Using default.");
		this->joint_pos_A = {0.0,-0.0364, 0.0, 0.5051, 0.0, 1.1522, 0.0};
		success = false;
	}


    return success;
}

bool TaskSequencer::call_example_task(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
    
    // Checking the request for correctness
    if(!req.data){
        ROS_WARN("Did you really want to call the simple grasp task service with data = false?");
        res.success = true;
        res.message = "The service call_simple_grasp_task done correctly with false request!";
        return true;
    }

    Eigen::Affine3d grasp_transform_aff; tf::poseMsgToEigen(this->grasp_T, grasp_transform_aff);
    Eigen::Affine3d pre_grasp_transform_aff; tf::poseMsgToEigen(this->pre_grasp_T, pre_grasp_transform_aff);

    geometry_msgs::Pose pre_grasp_pose; geometry_msgs::Pose grasp_pose;
    tf::poseEigenToMsg(grasp_transform_aff * pre_grasp_transform_aff, pre_grasp_pose);
    tf::poseEigenToMsg(grasp_transform_aff, grasp_pose);

    // Open the gripper
    
    this->OpenGripper(true);

    // Plan and go to Pre Grasp Pose 
    this->PlanAndExecutePose(pre_grasp_pose, false);

    // Plan and go to Grasp Pose
    this->PlanAndExecuteSlerp(grasp_pose, false);

    // Close the gripper

    this->CloseGripper(true);
    sleep(0.2);

    // Plan and go to Pre Grasp Pose

    this->PlanAndExecuteSlerp(pre_grasp_pose, false);

    // Plan and go to Joint Position A

    this->PlanAndExecuteJoint(joint_pos_A, true);

    // Now, everything finished well
    res.success = true;
    res.message = "The service call_example_task was correctly performed!";
    return true;   
}




void TaskSequencer::FirstPrimitive()
{
    std::vector<double>  joint_central_pose= {1.9278074234844063, 0.002317664744280634, -0.3296313546352003, 0.2832045471070792, 0.05554009737100163, 1.3062192448908974, -1.6572252727301944};
    std::vector<double>  rotation_home_gripper_position={-0.5609045918496092, -0.9876596595914524, 1.8417492350489582, 0.3072976210851497, -2.0765127407854536, -1.159086094720677, -2.390829265297107};
    std::vector<double>  release_dice_position={-0.6716783265204969, -0.42436470083887023, 2.382754633409796, 0.7007459544674743, 0.5079562429582833, 0.6484524826537372, -1.6661392821102945};
    
    
    
    geometry_msgs::Pose grasp_pose= geometry_msgs::Pose();
    geometry_msgs::Pose lift_pose= geometry_msgs::Pose();
    
    
    
    this->PlanAndExecuteJoint(joint_central_pose, true);
    geometry_msgs::PoseStamped dice_present_pose = geometry_msgs::PoseStamped();
    //dice_present_pose=this->dice_pose;


    //FOR THE REAL TASK READING THE POSITION FROM THE ROBOT
    dice_present_pose.pose.position.x = 0.002;
    dice_present_pose.pose.position.y = 0.373;
    dice_present_pose.pose.position.z = 0.131;

    // orientation
    dice_present_pose.pose.orientation.x = 0.362;
    dice_present_pose.pose.orientation.y = 0.932;
    dice_present_pose.pose.orientation.z = 0.000;
    dice_present_pose.pose.orientation.w = -0.001;
    

    // define the low position of the EE
    table_height = dice_present_pose.pose.position.z-0.01;
    
    release_dice_position_EE_low = release_dice_position_EE_high;
    release_dice_position_EE_low.position.z = this->table_height;

    geometry_msgs::Pose pre_grasp_pose= geometry_msgs::Pose();
    pre_grasp_pose=dice_present_pose.pose;
    pre_grasp_pose.position.z+=0.1;
    this->PlanAndExecutePose(pre_grasp_pose, false);
    this->OpenGripper(true);

    
    grasp_pose=dice_present_pose.pose;
    this->PlanAndExecuteSlerp(grasp_pose, false);
    sleep(0.3);
    this->CloseGripper(true);
    
    lift_pose=grasp_pose;
    ROS_INFO("grasp_pose: %f,%f,%f", grasp_pose.position.x,grasp_pose.position.y,grasp_pose.position.z);
    lift_pose.position.z+=0.1;
    ROS_INFO("grasp_pose: %f,%f,%f", grasp_pose.position.x,grasp_pose.position.y,grasp_pose.position.z);    
    this->PlanAndExecuteSlerp(lift_pose, false);
       // joint for sim
    //std::vector<double>  rotation_home_gripper_position={1.83513486091298, 0.0948283076414489, -0.2163687438229731, 0.6657989377844125, -0.013279136712920803, 0.7830107728531628, 0.11035854944976009};
    //this->PlanAndExecuteJoint(release_dice_position, true);

    // Go in the right pose before release the dice
    this->PlanAndExecuteSlerp(release_dice_position_EE_high, false);

    // Go in the right pose to release the dice near the table
    this->PlanAndExecuteSlerp(release_dice_position_EE_low, false);

    

    // geometry_msgs::Pose gripper_central_pose=performFK(rotation_home_gripper_position);
    // gripper_central_pose.position.z-=0.05;
    // this->PlanAndExecutePose(gripper_central_pose, false);
    // ROS_INFO("first move down:");
    // // geometry_msgs::Pose gripper_central_pose_low=geometry_msgs::Pose();//the position gripper releases the dice
    // // gripper_central_pose_low=gripper_central_pose;
    // // //gripper_central_pose_low.position.z=dice_present_pose.pose.position.z;
    // // gripper_central_pose_low.position.z-=0.01;
    // // ROS_INFO("gripper_central_pose_low: %f,%f,%f", gripper_central_pose_low.position.x,gripper_central_pose_low.position.y,gripper_central_pose_low.position.z);
    // gripper_central_pose.position.z-=0.05;
    // this->PlanAndExecuteSlerp(gripper_central_pose, false);
    // ROS_INFO("second move down:");
    this->OpenGripper(true);

    // move the robot to read data from the dice    
    this->PlanAndExecuteSlerp(this->read_pose, false);

}



geometry_msgs::Pose TaskSequencer::Rotate_Axis_x(bool clockwise,geometry_msgs::Pose current_pose)
{
    Eigen::Quaterniond mut_quat(0.9238795, 0.3826834, 0, 0); // anticlockwise
    geometry_msgs::Pose turn_pose= geometry_msgs::Pose();
    Eigen::Quaterniond grasp_quat(1.0, 0.0, 0.0, 0.0); 

    grasp_quat.w()=current_pose.orientation.w;
    grasp_quat.x()=current_pose.orientation.x;
    grasp_quat.y()=current_pose.orientation.y;
    grasp_quat.z()=current_pose.orientation.z;

    if (clockwise)
        Eigen::Quaterniond mut_quat(0.9238795, -0.3826834, 0, 0);
        // mut_quat.w() =0.9238795; 
        // mut_quat.x() = -0.3826834; 
        // mut_quat.y() = 0; 
        // mut_quat.z()= 0; 
        ROS_INFO("clockwise");
        ROS_INFO("%f",mut_quat.x());
    

    Eigen::Quaterniond result = grasp_quat * mut_quat;

    turn_pose=current_pose;
    turn_pose.orientation.x=result.x();
    turn_pose.orientation.y=result.y();
    turn_pose.orientation.z=result.z();
    turn_pose.orientation.w=result.w();
    return turn_pose;

}

geometry_msgs::Pose TaskSequencer::Rotate_Axis_x_positive(geometry_msgs::Pose current_pose)
{
    Eigen::Quaterniond mut_quat(0.9238795, 0.3826834, 0, 0); // anticlockwise
    geometry_msgs::Pose turn_pose= geometry_msgs::Pose();
    Eigen::Quaterniond grasp_quat(1.0, 0.0, 0.0, 0.0); 

    grasp_quat.w()=current_pose.orientation.w;
    grasp_quat.x()=current_pose.orientation.x;
    grasp_quat.y()=current_pose.orientation.y;
    grasp_quat.z()=current_pose.orientation.z;

    Eigen::Quaterniond result = grasp_quat * mut_quat;

    turn_pose=current_pose;
    turn_pose.orientation.x=result.x();
    turn_pose.orientation.y=result.y();
    turn_pose.orientation.z=result.z();
    turn_pose.orientation.w=result.w();
    return turn_pose;
}
geometry_msgs::Pose TaskSequencer::Rotate_Axis_x_negative(geometry_msgs::Pose current_pose)
{
    Eigen::Quaterniond mut_quat(0.9238795, -0.3826834, 0, 0); // anticlockwise
    geometry_msgs::Pose turn_pose= geometry_msgs::Pose();
    Eigen::Quaterniond grasp_quat(1.0, 0.0, 0.0, 0.0); 

    grasp_quat.w()=current_pose.orientation.w;
    grasp_quat.x()=current_pose.orientation.x;
    grasp_quat.y()=current_pose.orientation.y;
    grasp_quat.z()=current_pose.orientation.z;

    Eigen::Quaterniond result = grasp_quat * mut_quat;

    turn_pose=current_pose;
    turn_pose.orientation.x=result.x();
    turn_pose.orientation.y=result.y();
    turn_pose.orientation.z=result.z();
    turn_pose.orientation.w=result.w();
    return turn_pose;
}

geometry_msgs::Pose TaskSequencer::Rotate_Axis_z(bool clockwise,geometry_msgs::Pose current_pose)
{
    Eigen::Quaterniond mut_quat( 0.9238795, 0, 0, 0.3826834); // anticlockwise


    geometry_msgs::Pose turn_pose= geometry_msgs::Pose();
    Eigen::Quaterniond grasp_quat(1.0, 0.0, 0.0, 0.0); 

    grasp_quat.w()=current_pose.orientation.w;
    grasp_quat.x()=current_pose.orientation.x;
    grasp_quat.y()=current_pose.orientation.y;
    grasp_quat.z()=current_pose.orientation.z;

    if (clockwise)
        mut_quat.w() = 0.9238795;
        mut_quat.x() = 0; 
        mut_quat.y() = 0; 
        mut_quat.z()= -0.3826834; 

    Eigen::Quaterniond result = grasp_quat * mut_quat;

    turn_pose=current_pose;
    turn_pose.orientation.x=result.x();
    turn_pose.orientation.y=result.y();
    turn_pose.orientation.z=result.z();
    turn_pose.orientation.w=result.w();
    return turn_pose;

}




void TaskSequencer::SecondPrimitive_rotx(){
    geometry_msgs::Pose grasp_pose_oriented_high= geometry_msgs::Pose();
    //geometry_msgs::Pose grasp_pose_oriented_low= geometry_msgs::Pose();
    bool isClockwise;

    isClockwise = true;

    // ----------------- todo. clockwise or not -----------------

    // define the pose to take the dice--> high position and with the EE oriented in right way
    grasp_pose_oriented_high = Rotate_Axis_x(isClockwise,release_dice_position_EE_high);

    // go over the dice
    this->PlanAndExecutePose(grasp_pose_oriented_high, false);

    // go near the dice
    // we have to set the z coordinate equal to the one of the board
    grasp_pose_oriented_low = grasp_pose_oriented_high;
    grasp_pose_oriented_low.position.z = this-> table_height;
    this->PlanAndExecuteSlerp(grasp_pose_oriented_low, false);

    // take the dice
    this->CloseGripper(true);


    // higher the robot
    this->PlanAndExecuteSlerp(grasp_pose_oriented_high, false);


    // turn the dice - 2 times to obtain 90 degrees 
    // // to do HOW MANY TIMES -----------------
    //isClockwise = false;
    // grasp_pose_oriented_high = Rotate_Axis_x(isClockwise,grasp_pose_oriented_high);
    // this->PlanAndExecutePose(grasp_pose_oriented_high, false);

    // grasp_pose_oriented_high = Rotate_Axis_x(isClockwise,grasp_pose_oriented_high);
    // this->PlanAndExecutePose(grasp_pose_oriented_high, false);

    grasp_pose_oriented_high = Rotate_Axis_x_negative(grasp_pose_oriented_high);
    this->PlanAndExecutePose(grasp_pose_oriented_high, false);

    grasp_pose_oriented_high = Rotate_Axis_x_negative(grasp_pose_oriented_high);
    this->PlanAndExecutePose(grasp_pose_oriented_high, false);

    // grasp_pose_oriented_high = Rotate_Axis_x_positive(grasp_pose_oriented_high);
    // this->PlanAndExecutePose(grasp_pose_oriented_high, false);



    


    // go near the table to release the dice
    this->grasp_pose_oriented_low.position.z = this->table_height;
    this->grasp_pose_oriented_low.orientation.x = grasp_pose_oriented_high.orientation.x;
    this->grasp_pose_oriented_low.orientation.y = grasp_pose_oriented_high.orientation.y;
    this->grasp_pose_oriented_low.orientation.z = grasp_pose_oriented_high.orientation.z;
    this->grasp_pose_oriented_low.orientation.w = grasp_pose_oriented_high.orientation.w;
    this->PlanAndExecuteSlerp(grasp_pose_oriented_low, false);
    
    // lift the dice
    this->OpenGripper(true);

    // remove the robot from the camera field of view
    this->PlanAndExecuteSlerp(this->read_pose, false);

}

void TaskSequencer::ThirdPrimitive_rotz(){

    geometry_msgs::Pose dice_present_pose= geometry_msgs::Pose();
    geometry_msgs::Pose pre_grasp_pose= geometry_msgs::Pose(); // high pose over the dice
    geometry_msgs::Pose final_dice_pose= geometry_msgs::Pose();
    geometry_msgs::Pose high_robot_pose= geometry_msgs::Pose();



    //FOR THE REAL TASK READING THE POSITION FROM THE ROBOT
    // dice_present_pose.position.x = 0.002;
    // dice_present_pose.position.y = 0.373;
    // dice_present_pose.position.z = 0.131;

    // // orientation
    // dice_present_pose.orientation.x = 0.362;
    // dice_present_pose.orientation.y = 0.932;
    // dice_present_pose.orientation.z = 0.000;
    // dice_present_pose.orientation.w = -0.001;

    dice_present_pose = this->grasp_pose_oriented_low;

    bool isClockwise;
    isClockwise = true;


    // read dice pose
    //dice_present_pose=this->dice_pose.pose;
    table_height = dice_present_pose.position.z;

    pre_grasp_pose=dice_present_pose;
    pre_grasp_pose.position.z+=0.05;
    this->PlanAndExecutePose(pre_grasp_pose, false);
    //this->OpenGripper(true);

    // go near to the dice
    this->PlanAndExecuteSlerp(dice_present_pose, false);

    // close the gripper
    this ->CloseGripper(true);

    // compute the position
    final_dice_pose = Rotate_Axis_z(isClockwise,dice_present_pose);
    this->PlanAndExecutePose(final_dice_pose, false);

    final_dice_pose = Rotate_Axis_z(isClockwise,final_dice_pose);
    this->PlanAndExecutePose(final_dice_pose, false);

    this ->OpenGripper(true);

    // Lift the robot before moving it
    high_robot_pose = final_dice_pose;
    high_robot_pose.position.z+=0.1;
    this->PlanAndExecutePose(high_robot_pose, false);

    // remove the robot from the camera field of view
    this->PlanAndExecuteSlerp(this->read_pose, false);

}

bool TaskSequencer::Is_oppsite(int current_dice_upward,int target_dice_upward)
{
    if (current_dice_upward==1 && target_dice_upward==6)
    {
        return true;
    }
    if (current_dice_upward==2 && target_dice_upward==5)
    {
        return true;
    }
    if (current_dice_upward==3 && target_dice_upward==4)
    {
        return true;
    }
    return false;
}



// Callback for template task service
bool TaskSequencer::call_template_task(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
    
    bool NumberFound = false;
    int target_dice_upward=3;
    int current_dice_upward=5;//get current dice upward from camera

    // Checking the request for correctness

    if(!req.data){
        ROS_WARN("Did you really want to call the template task service with data = false?");
        res.success = true;
        res.message = "The service call_template_task done correctly with false request!";
        return true;
    }
    
    // define the position in which put the dice before turning it
    this->release_dice_position_EE_high.position.x=0.011;
    this->release_dice_position_EE_high.position.y=0.317;
    this->release_dice_position_EE_high.position.z=0.181;
    this->release_dice_position_EE_high.orientation.w=0.030;
    this->release_dice_position_EE_high.orientation.x=0.728;
    this->release_dice_position_EE_high.orientation.y=0.685;
    this->release_dice_position_EE_high.orientation.z=-0.008;


    // define the position to put the robot to read the dice
    read_pose = this->release_dice_position_EE_high;
    read_pose.position.x = 0.2;



    // /**/coding here
    /*Compose the high level task by using the PlanAndExecutePose, PlanAndExecuteJoint
    and PlanAndExecuteSlerp functions*/
    sleep(3);
    // this->PlanAndExecutePose(release_dice_position_EE_high, false);
    // geometry_msgs::Pose test_pose_1= geometry_msgs::Pose();
    // geometry_msgs::Pose test_pose_2= geometry_msgs::Pose();
    // test_pose_1 = Rotate_Axis_x(false,release_dice_position_EE_high);
    // this->PlanAndExecutePose(test_pose_1, false);
    // test_pose_2 = Rotate_Axis_x(true,test_pose_1);
    // this->PlanAndExecutePose(test_pose_1, false);
    // test_pose_2 = Rotate_Axis_x(true,test_pose_2);
    // this->PlanAndExecutePose(test_pose_1, false);

    //this->OpenGripper(true);
    //this->PlanAndExecuteJoint(this->rotation_home_gripper_position , true);

    //std::vector<double>  joint_central_pose={0.03513486091298, 0.0948283076414489, -0.2163687438229731, 0.6657989377844125, -0.013279136712920803, 0.7830107728531628, 0.11035854944976009};
    //this->PlanAndExecuteJoint(joint_central_pose, true);
    // moveit::planning_interface::MoveGroupInterface move_group_interface_a_bot(this->group_name);
    // geometry_msgs::PoseStamped tp1 = move_group_interface_a_bot.getCurrentPose();
    // ROS_INFO("tp1: %f,%f,%f", tp1.pose.position.x,tp1.pose.position.y,tp1.pose.position.z);
    // geometry_msgs::Pose tp2= geometry_msgs::Pose();
    // tp2.position.x+=-0.2;
    // this->PlanAndExecutePose(tp2, false);


    // put the dice in a comfortable position for tunring it with the robot

    this->FirstPrimitive();
    this->SecondPrimitive_rotx();

    // this->ThirdPrimitive_rotz();

    // this->SecondPrimitive_rotx();

    
    // while(!NumberFound)
    // {
    //     //get current dice upward from camera
    //     if (current_dice_upward==target_dice_upward)
    //     {
    //         NumberFound=true;
    //     }
    //     else if (Is_oppsite(current_dice_upward,target_dice_upward))
    //     {
    //         this->SecondPrimitive_rotx();
    //         this->SecondPrimitive_rotx();
    //         NumberFound=true;
    //     }
    //     else
    //     {
    //         this->SecondPrimitive_rotx();
    //         //get current dice upward from camera
    //         if (current_dice_upward==target_dice_upward)
    //         {
    //             NumberFound=true;
    //         }
    //         else if (Is_oppsite(current_dice_upward,target_dice_upward))
    //         {
    //             this->SecondPrimitive_rotx();
    //             this->SecondPrimitive_rotx();
    //             NumberFound=true;
    //         }
    //         else
    //         {
    //             this->ThirdPrimitive_rotz();
    //             this->SecondPrimitive_rotx();
    //             //get current dice upward from camera
    //             if (current_dice_upward==target_dice_upward)
    //             {
    //                 NumberFound=true;
    //             }
    //             else
    //             {
    //                 this->SecondPrimitive_rotx();
    //                 this->SecondPrimitive_rotx();
    //                 NumberFound=true;
    //             }

    //         }

    //     }

    // }
        // read the dice 

        // if the right number 
        //NumberFound = false;

        // turn the dice 
        // if (true)
        //     this->SecondPrimitive_rotx();

        // else
        //     this->ThirdPrimitive_rotz();
        


//     geometry_msgs::PoseStamped dice_present_pose = geometry_msgs::PoseStamped();
//     // dice_present_pose=this->dice_pose;
//     // dice_present_pose.pose.position.x=-0.018;
//     // dice_present_pose.pose.position.y=0.374;
//     // dice_present_pose.pose.position.z=0.119;

//     // dice_present_pose.pose.orientation.x=0.685;
//     // dice_present_pose.pose.orientation.y=0.729;
//     // dice_present_pose.pose.orientation.z=0.013;
//     // dice_present_pose.pose.orientation.w=0.003;



//     geometry_msgs::Pose pre_grasp_pose= geometry_msgs::Pose();
//     geometry_msgs::Pose grasp_pose= geometry_msgs::Pose();
//     geometry_msgs::Pose central_pose= geometry_msgs::Pose();
//     geometry_msgs::Pose lift_pose= geometry_msgs::Pose();
//     geometry_msgs::Pose grip_central_pose_high= geometry_msgs::Pose();
//     geometry_msgs::Pose grip_central_pose_low= geometry_msgs::Pose();

//     geometry_msgs::Pose turn_pose= geometry_msgs::Pose();


//     joint for real robot
//     std::vector<double>  joint_central_pose= {1.549736581526768, -0.08227518272944678, 0.16259978305273545, 0.6065216890391233, -0.058348136751305964, 1.0301903381592425, 0.1746527840932433};



//     grasp_pose=dice_present_pose.pose;
//     pre_grasp_pose=dice_present_pose.pose;
//     central_pose=grasp_pose;
//     lift_pose=grasp_pose;
//     lift_pose.position.z+=0.05;
//     // central_pose.position.x=0.0250864;
//     // central_pose.position.y=-0.03432;
//     central_pose.position.x=dice_present_pose.pose.position.x+0.01;
//     central_pose.position.y=dice_present_pose.pose.position.y+0.03;


//     pre_grasp_pose.position.z+=0.05;


//     Eigen::Quaterniond grasp_quat(1.0, 2.0, 3.0, 4.0); 
//     grasp_quat.w()=grasp_pose.orientation.w;
//     grasp_quat.x()=grasp_pose.orientation.x;
//     grasp_quat.y()=grasp_pose.orientation.y;
//     grasp_quat.z()=grasp_pose.orientation.z;

//     Eigen::Quaterniond mut_quat(0.9238795, 0.3826834, 0, 0); 

//     Eigen::Quaterniond result = grasp_quat * mut_quat;

//     turn_pose=pre_grasp_pose;
//     turn_pose.orientation.x=result.x();
//     turn_pose.orientation.y=result.y();
//     turn_pose.orientation.z=result.z();
//     turn_pose.orientation.w=result.w();

    
//     ROS_INFO("pre_grasp_pose: %f,%f,%f", this->dice_pose.pose.position.x,this->dice_pose.pose.position.y,this->dice_pose.pose.position.z);

//     sleep(1);
  
//     this->PlanAndExecutePose(pre_grasp_pose, false);

//     this->PlanAndExecuteSlerp(grasp_pose, false);

//     sleep(1);

// //catch the dice
//     this->CloseGripper(true);

//     // put the dice in the central pose
//     this->PlanAndExecuteJoint(joint_central_pose, true);


//     //leave the dice
    



    
//     this->PlanAndExecuteSlerp(pre_grasp_pose, false);
  
//     this->PlanAndExecutePose(turn_pose, false);



















    // this->PlanAndExecuteSlerp(lift_pose, false);

    // this->PlanAndExecuteJoint(joint_central_pose, true);

    // while (/* condition */)
    // {
    //     /* code */
    // }
    // sleep(5);
    
    // ROS_INFO("Dice: %f", this->dice_pose.pose.position.x);
    // ROS_INFO("Dice: %f", this->dice_pose.pose.position.x);
    // ROS_INFO("Dice: %f", this->dice_pose.pose.position.x);
    // ROS_INFO("Dice: %f", this->dice_pose.pose.position.x);
    // ROS_INFO("Dice: %f", this->dice_pose.pose.position.x);

    // sleep(1);
    // this->CloseGripper(true);





    /**/

    /*REMEMBER: if you want to create additional task functions like that, you have to declare 
    those in "TaskSequencer.h" (see the declaration of "call_template_task" at line 52 if you do 
    not know how to do it)*/
    
    // Now, everything finished well
    res.success = true;
    res.message = "The service call_template_task was correctly performed!";
    return true;
}

bool TaskSequencer::PlanAndExecutePose(geometry_msgs::Pose& pose, bool is_relative){
    
    std_srvs::SetBool set_bool_srv;

    // Setting zero pose as starting from present

    geometry_msgs::Pose present_pose = geometry_msgs::Pose();
    present_pose.position.x = 0.0; present_pose.position.y = 0.0; present_pose.position.z = 0.0;
    present_pose.orientation.x = 0.0; present_pose.orientation.y = 0.0; present_pose.orientation.z = 0.0; present_pose.orientation.w = 1.0;

    /* PLAN 1: Plan to POSE */

    if(!this->abb_client.call_pose_service(pose, present_pose, is_relative, this->tmp_traj_arm, this->tmp_traj_arm)){
        ROS_ERROR("Could not plan to the specified pre grasp pose.");
        set_bool_srv.response.success = false;
        set_bool_srv.response.message = "The service call_pose_service was NOT performed correctly!";
        return false;
    }  

    /* EXEC 1: Going to POSE*/

    if(!this->abb_client.call_arm_control_service(this->tmp_traj_arm)){
        ROS_ERROR("Could not go to PreGraspPose.");
        set_bool_srv.response.success = false;
        set_bool_srv.response.message = "The service call_arm_control_service was NOT performed correctly! Error in arm control.";
        return false;
    }

    /* WAIT 1: Wait to finish the task*/

    if(!this->abb_client.call_arm_wait_service(this->waiting_time)){ // WAITING FOR END EXEC
        ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to Pre Grasp Pose");
        set_bool_srv.response.success = false;
        set_bool_srv.response.message = "The service call_arm_wait_service was NOT performed correctly! Error wait in arm control.";
        return false;
    }
    return set_bool_srv.response.success = true;
}

bool TaskSequencer::PlanAndExecuteJoint(std::vector<double>& joint_goal, bool flag_state){
    
    std_srvs::SetBool set_bool_srv;

    /* PLAN 1: Plan to JOINT Position */

    if(!this->abb_client.call_joint_service(joint_goal, flag_state, this->tmp_traj_arm, this->tmp_traj_arm)){
        ROS_ERROR("Could not plan to the specified JOINT position.");
        set_bool_srv.response.success = false;
        set_bool_srv.response.message = "The service call_joint_service was NOT performed correctly!";
        return false;
    }  

    /* EXEC 1: Going to Joint*/

    if(!this->abb_client.call_arm_control_service(this->tmp_traj_arm)){
        ROS_ERROR("Could not go to JOINT position.");
        set_bool_srv.response.success = false;
        set_bool_srv.response.message = "The service call_arm_control_service was NOT performed correctly! Error in arm control.";
        return false;
    }

    /* WAIT 1: Wait to finish the task*/

    if(!this->abb_client.call_arm_wait_service(this->waiting_time)){ // WAITING FOR END EXEC
        ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to Pre Grasp Pose");
        set_bool_srv.response.success = false;
        set_bool_srv.response.message = "The service call_arm_wait_service was NOT performed correctly! Error wait in arm control.";
        return false;
    }
    return set_bool_srv.response.success = true;
}

bool TaskSequencer::PlanAndExecuteSlerp(geometry_msgs::Pose& pose, bool is_relative){

    std_srvs::SetBool set_bool_srv;

    // Setting zero pose as starting from present

    geometry_msgs::Pose present_pose = geometry_msgs::Pose();
    present_pose.position.x = 0.0; present_pose.position.y = 0.0; present_pose.position.z = 0.0;
    present_pose.orientation.x = 0.0; present_pose.orientation.y = 0.0; present_pose.orientation.z = 0.0; present_pose.orientation.w = 1.0;

    /* PLAN 1: Plan to POSE */

    if(!this->abb_client.call_slerp_service(pose, present_pose, is_relative, this->tmp_traj_arm, this->tmp_traj_arm)){
        ROS_ERROR("Could not plan to the specified pose.");
        set_bool_srv.response.success = false;
        set_bool_srv.response.message = "The service call_slerp_service was NOT performed correctly!";
        return false;
    }  

    /* EXEC 1: Going to POSE*/

    if(!this->abb_client.call_arm_control_service(this->tmp_traj_arm)){
        ROS_ERROR("Could not go to pose.");
        set_bool_srv.response.success = false;
        set_bool_srv.response.message = "The service call_arm_control_service was NOT performed correctly! Error in arm control.";
        return false;
    }

    /* WAIT 1: Wait to finish the task*/

    if(!this->abb_client.call_arm_wait_service(this->waiting_time)){ // WAITING FOR END EXEC
        ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to Pre Grasp Pose");
        set_bool_srv.response.success = false;
        set_bool_srv.response.message = "The service call_arm_wait_service was NOT performed correctly! Error wait in arm control.";
        return false;
    }
    return set_bool_srv.response.success = true;
}

bool TaskSequencer::CloseGripper(bool close){
    
    std_srvs::SetBool set_bool_srv;

    if(!this->abb_client.call_closing_gripper(close)){
        ROS_ERROR("Could not close the gripper.");
        set_bool_srv.response.success = false;
        set_bool_srv.response.message = "The service call_closing_gripper was NOT performed correctly!";
        return false;
    }  
    return set_bool_srv.response.success = true;
}

bool TaskSequencer::OpenGripper(bool open){

    std_srvs::SetBool set_bool_srv;

    if(!this->abb_client.call_opening_gripper(open)){
        ROS_ERROR("Could not open the gripper.");
        set_bool_srv.response.success = false;
        set_bool_srv.response.message = "The service call_opening_gripper was NOT performed correctly!";
        return false;
    }  
    return set_bool_srv.response.success = true;
}

// Convert xyzrpy vector to geometry_msgs Pose
geometry_msgs::Pose TaskSequencer::convert_vector_to_pose(std::vector<double> input_vec){
    
    // Creating temporary variables
    geometry_msgs::Pose output_pose;
    Eigen::Affine3d output_affine;

    // Getting translation and rotation
    Eigen::Vector3d translation(input_vec[0], input_vec[1], input_vec[2]);
    output_affine.translation() = translation;
    Eigen::Matrix3d rotation = Eigen::Matrix3d(Eigen::AngleAxisd(input_vec[5], Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(input_vec[4], Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(input_vec[3], Eigen::Vector3d::UnitX()));
    output_affine.linear() = rotation;    
    
    // Converting to geometry_msgs and returning
    tf::poseEigenToMsg(output_affine, output_pose);
    return output_pose;
}

// FK and IK Functions which makes use of MoveIt
geometry_msgs::Pose TaskSequencer::performFK(std::vector<double> joints_in){
    const robot_state::JointModelGroup* joint_model_group = this->kinematic_model->getJointModelGroup(group_name);
    this->kinematic_state->setJointGroupPositions(joint_model_group, joints_in);
    const Eigen::Affine3d& end_effector_eigen = this->kinematic_state->getGlobalLinkTransform(end_effector_name);
    geometry_msgs::Pose end_effector_pose;
    tf::poseEigenToMsg(end_effector_eigen, end_effector_pose);
    return end_effector_pose;
}

bool TaskSequencer::performIK(geometry_msgs::Pose pose_in, double timeout, std::vector<double>& joints_out){
    Eigen::Isometry3d end_effector_state;
    tf::poseMsgToEigen(pose_in, end_effector_state);
    const robot_state::JointModelGroup* joint_model_group = this->kinematic_model->getJointModelGroup(group_name);
    bool found_ik = this->kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);

    if (!found_ik){
        ROS_ERROR("Could not find IK solution in TaskSequencer...");
        return false;
    }

    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    kinematic_state->copyJointGroupPositions(joint_model_group, joints_out);
    this->kinematic_state->copyJointGroupPositions(joint_model_group, joints_out);

    if (DEBUG){
        ROS_INFO("Found an IK solution in TaskSequencer: ");
        for (std::size_t i = 0; i < joint_names.size(); ++i){
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joints_out[i]);
        }
    }

    return true;
}
