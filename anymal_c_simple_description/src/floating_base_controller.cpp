#include <pluginlib/class_list_macros.h>
#include "floating_base_controller.h" //library of the computed torque 

namespace my_controller_ns{

bool floating_base_controller::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
{ 
    this->n = n;
    //Creazione del modello
    model = new Model();
    
    string path;
    if (!n.getParam("/path", path)) 
    {
        ROS_ERROR("Specify the path of model.urdf");
        exit(1);
    }
    

    if (!Addons::URDFReadFromFile("/home/david/catkin_ws/src/anymal_c_simple_description/urdf/model.urdf", model, true, false))
    {
        cout << "Error loading model" << endl;
        abort();
    }
   
    
    cout <<Utils::GetModelDOFOverview(*model)<< endl;
   
    cout << model->dof_count << endl; 
   
    M_ =  Eigen::MatrixXd::Zero(18,18);         
    c_ =  Eigen::VectorXd::Zero(18);                                      
    g_ =  Eigen::VectorXd::Zero(18);         
    cor_ = Eigen::VectorXd::Zero(18); 
    S_ = Eigen::MatrixXd::Zero(12,18); 
    Nc_ = Eigen::MatrixXd::Zero(18,18);  
    Jc_ = Eigen::MatrixXd::Zero(12,18);     

    int k=6;
    
    for (size_t i = 0; i<12; i++)
    {
        S_(i,k) = 1;
            k++;
    }

    /* Inizializing the Kp and Kv gains */
    /*
    if (!n.getParam("kp",kp) || !n.getParam("kv", kv)) {
        ROS_ERROR("Could not get parameter kpi or kv!");
        return false;
    }
    */

    std::vector<std::string> joint_names;
    if (!n.getParam("joint_names", joint_names) || joint_names.size() != 12) 
    {
        ROS_ERROR("Error in parsing joints name!");
        return false;
    }

        for (size_t i = 0; i<12; i++)
        {   
            joint_handle.push_back(hw->getHandle(joint_names[i])); 
            q_curr(i+6) = joint_handle[i].getPosition();
            dot_q_curr(i+6) = joint_handle[i].getVelocity();
        }

    
    this->sub_command_ = n.subscribe<sensor_msgs::JointState> ("command", 1, &floating_base_controller::setCommandCB, this);  
    this-> sub_gaze = n.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 1, &floating_base_controller::state_estimator, this);
     
    //this->pub_err_ = n.advertise<sensor_msgs::JointState> ("tracking_error", 1);
    
    return true;
}
    
    void floating_base_controller::starting(const ros::Time& time){ 

        

    }

    void floating_base_controller::stopping(const ros::Time& time){ }
    
    void floating_base_controller::update(const ros::Time& time, const ros::Duration& period)
    {
       tf::Quaternion q(q_temp[3], q_temp[4], q_temp[5], q_temp[6]);
       tf::Matrix3x3 m(q);
       m.getRPY(roll,pitch,yaw);
       rpy_temp << roll, pitch, yaw;
       
       
        for (size_t i = 0; i<3; i++)
        {
            q_curr(i) = q_temp(i);
           
        }
        for (size_t i = 0; i<3; i++)
        {
            q_curr(i+3)= rpy_temp(i);
           
        }


        for (size_t i = 0; i<6; i++)
        {
            dot_q_curr(i) = dot_q_temp(i);
        }

        for (size_t i = 0; i<12; i++)
        {
            q_curr(i+6) = joint_handle[i].getPosition();
            dot_q_curr(i+6) = joint_handle[i].getVelocity();
        }

        cout << "q_curr update"<< endl<< q_curr << endl;
        
        VectorNd index_link = VectorNd::Zero(4);

        index_link(0) = model->GetBodyId("LF_FOOT");
        index_link(1) = model->GetBodyId("RF_FOOT");
        index_link(2) = model->GetBodyId("LH_FOOT");
        index_link(3) = model->GetBodyId("RH_FOOT");

        
        cout<<"Index vector of the link"<< endl;
        cout<< index_link << endl;
   
        Eigen::MatrixXd J_c1 = Eigen::MatrixXd::Zero(3, 18);
        Eigen::MatrixXd J_c2 = Eigen::MatrixXd::Zero(3, 18);
        Eigen::MatrixXd J_c3 = Eigen::MatrixXd::Zero(3, 18);
        Eigen::MatrixXd J_c4 = Eigen::MatrixXd::Zero(3, 18);
   
        CalcPointJacobian(*model, q_curr, index_link(0), Vector3d(0., 0., 0.), J_c1);
    
        CalcPointJacobian(*model, q_curr, index_link(1), Vector3d(0., 0., 0.), J_c2);
      
        CalcPointJacobian(*model, q_curr, index_link(2), Vector3d(0., 0., 0.), J_c3);
      
        CalcPointJacobian(*model, q_curr, index_link(3), Vector3d(0., 0., 0.), J_c4);
      

        Jc_ << J_c1, J_c2, J_c3, J_c4;
      
        cout << Jc_<< endl; 
        
        CompositeRigidBodyAlgorithm(*model, q_curr, M_); 
        
        cout << "Inertia matrix:" << std::endl;
        cout << M_<< endl;
        
        NonlinearEffects(*model, q_curr, dot_q_curr, c_);
        
        VectorNd Q = VectorNd::Zero (18);
        
        NonlinearEffects(*model, q_curr, Q, g_);
        
        cout << "Gravity vector:" << endl;
        cout << g_ << endl; 
        
        cor_ = c_ - g_;
        
        cout << "Coriolis*dot_Q " << endl;
        cout << cor_ << endl;

        //Proiettore nel nullo del jacobiano dei contatti

        Eigen::MatrixXd Id = Eigen::MatrixXd::Identity(18,18);
        Eigen::MatrixXd M_inv = M_.inverse();
        Eigen::MatrixXd Jc_trans = Jc_.transpose(); 
        Eigen::MatrixXd  X = (Jc_ * M_inv * Jc_trans).inverse(); 
        Nc_ = Id - M_inv * Jc_trans * X * Jc_;
        Eigen::MatrixXd Nc_trans = Nc_.transpose();
        Eigen::MatrixXd S_trans = S_.transpose();
        Eigen::MatrixXd p = (Nc_trans * S_trans);
        Eigen::MatrixXd pinv= p.completeOrthogonalDecomposition().pseudoInverse();



        err = command_q_d - q_curr;
        dot_err = command_dot_q_d - dot_q_curr;


        cout << " q_curr"<< endl << q_curr <<endl;

        //abort();

        kp = 120;
        kv = 10;

        tau_cmd = pinv * Nc_trans * (M_ * (kp*err + kv*dot_err) + cor_ + g_);
    
        for (size_t i=0; i<12; i++)
        {
            joint_handle[i].setCommand(tau_cmd[i]);
        }
    }
      void floating_base_controller::setCommandCB(const sensor_msgs::JointStateConstPtr& msg)
     {
        command_q_d = Eigen::Map<const Eigen::Matrix<double, 18, 1>>((msg->position).data());
        command_dot_q_d = Eigen::Map<const Eigen::Matrix<double, 18, 1>>((msg->velocity).data());
    }


      void floating_base_controller::state_estimator(const gazebo_msgs::LinkStatesConstPtr& msg)
      {
        q_temp[0] = msg->pose[1].position.x;
        q_temp[1] = msg->pose[1].position.y;
        q_temp[2] = msg->pose[1].position.z;
        q_temp[3] = msg->pose[1].orientation.x;
        q_temp[4] = msg->pose[1].orientation.y;
        q_temp[5] = msg->pose[1].orientation.z;
        q_temp[6] =  msg->pose[1].orientation.w;

        dot_q_temp[0] = msg->twist[1].linear.x;
        dot_q_temp[1] = msg->twist[1].linear.y;
        dot_q_temp[2] = msg->twist[1].linear.z;
        dot_q_temp[3] = msg->twist[1].angular.x;
        dot_q_temp[4] = msg->twist[1].angular.y;
        dot_q_temp[5] = msg->twist[1].angular.z;

    }


    PLUGINLIB_EXPORT_CLASS(my_controller_ns::floating_base_controller, controller_interface::ControllerBase);
}