#include <pluginlib/class_list_macros.h>
#include "floating_base_controller.h" //library of the computed torque 
#include "pseudo_inversion.h"

namespace my_controller_ns{

bool floating_base_controller::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
{ 
    this->n = n;

    //Creazione del modello
    model = new Model();

    model2 = new Model();
    
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
   
    if (!Addons::URDFReadFromFile("/home/david/catkin_ws/src/anymal_c_simple_description/urdf/model2.urdf", model2, true, false))
    {
        cout << "Error loading model2" << endl;
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
    /*
      for (size_t i = 0; i<6; i++)
        {   
            command_q_d(i) = 0;
            command_dot_q_d(i) = 0;
        }
        for (size_t i = 0; i<12; i++)
        {   
            command_q_d(i+6) = 0;
            command_dot_q_d(i+6) = 0;
        }
        command_q_d(2) =  0.62;
    */
        
    command_q_d(0) =  0;
    command_q_d(1) =  0;
    command_q_d(2) =  0.628;
    command_q_d(3) =  0;
    command_q_d(4) =  0;
    command_q_d(5) =  0;
    command_q_d(6) =  0.00217255;
    command_q_d(7) =  0.00437822;
    command_q_d(8) =  -0.0109831;
    command_q_d(9) =  0.00221891;
    command_q_d(10) =  -0.00425637;
    command_q_d(11) =  0.0111751;
    command_q_d(12) =  -0.00206866;
    command_q_d(13) =  0.00430875;
    command_q_d(14) =   -0.0108892;
    command_q_d(15) =  -0.00261087;
    command_q_d(16) =  -0.00415898;
    command_q_d(17) =  0.0111347;
    

    cout << "command_q_d:" << endl;
    cout << command_q_d << endl;

    cout << "command_dot_q_d:" << endl;
    cout << command_dot_q_d << endl;


    return true;
}
    
    void floating_base_controller::starting(const ros::Time& time){ 

        

    }

    void floating_base_controller::stopping(const ros::Time& time){ }
    
    void floating_base_controller::update(const ros::Time& time, const ros::Duration& period)
    {   
        Eigen::MatrixXd M_ =  Eigen::MatrixXd::Zero(18, 18);

        cout << "Inertia matrix:" << std::endl;
        cout << M_<< endl;

        Eigen::VectorXd c_ = Eigen::VectorXd::Zero(18, 1); 
        Eigen::VectorXd g_ = Eigen::VectorXd::Zero(18, 1);

        cout << "Gravity vector:" << endl;
        cout << g_ << endl;
        
        for (size_t i = 0; i<18; i++)
        {
            q_curr(i) = 0;
            dot_q_curr(i) = 0;
            dot_dot_q_curr(i) = 0;
        }
        cout << "q_curr"<< endl<< q_curr << endl;
        cout << "dot_q_curr"<< endl<< dot_q_curr << endl;
        cout << "dot_dot_q_curr"<< endl<< dot_dot_q_curr << endl;

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
        index_link(1) = model->GetBodyId("LH_FOOT");
        index_link(2) = model->GetBodyId("RF_FOOT");
        index_link(3) = model->GetBodyId("RH_FOOT");

        
   
        Eigen::MatrixXd J_c1 = Eigen::MatrixXd::Zero(3, 18);
        Eigen::MatrixXd J_c2 = Eigen::MatrixXd::Zero(3, 18);
        Eigen::MatrixXd J_c3 = Eigen::MatrixXd::Zero(3, 18);
        Eigen::MatrixXd J_c4 = Eigen::MatrixXd::Zero(3, 18);

        Eigen::Matrix<double, 3, 1> v1;

        v1 << 0.0 , 0.0, 0.0;
   
        CalcPointJacobian(*model2, q_curr, index_link(0), v1, J_c1);
    
        CalcPointJacobian(*model2, q_curr, index_link(1), v1, J_c2);
      
        CalcPointJacobian(*model2, q_curr, index_link(2), v1, J_c3);
      
        CalcPointJacobian(*model2, q_curr, index_link(3), v1, J_c4);
      

        Jc_ << J_c1, J_c2, J_c3, J_c4;
      
        cout << "jacobian "<< endl << Jc_<< endl; 
        
        //MatrixNd M_ = MatrixNd::Zero(18, 18); 

        CompositeRigidBodyAlgorithm(*model, q_curr, M_); 
        
        cout << "Inertia matrix:" << std::endl;
        cout << M_<< endl;
        
        
        NonlinearEffects(*model, q_curr, dot_q_curr, c_);

        VectorNd QDot = VectorNd::Zero (18);
    
        NonlinearEffects(*model, q_curr, QDot, g_);
        
        cout << "Gravity vector:" << endl;
        cout << g_ << endl;
    
        cor_ = c_ - g_;

        //cout << "Coriolis*dot_Q " << endl;
        //cout << cor_ << endl;


        //Proiettore nel nullo del jacobiano dei contatti

        Eigen::MatrixXd Id = Eigen::MatrixXd::Identity(18,18);
        Eigen::MatrixXd M_inv = M_.inverse();
        Eigen::MatrixXd Jc_trans = Jc_.transpose(); 
        Eigen::MatrixXd  Y = (Jc_ * M_inv * Jc_trans);
        Eigen::MatrixXd  X = Y.inverse(); 
        Nc_ = Id - (M_inv * Jc_trans * X * Jc_);
        Eigen::MatrixXd Nc_trans = Nc_.transpose();
        Eigen::MatrixXd S_trans = S_.transpose();
        Eigen::MatrixXd p = (Nc_trans * S_trans);
        Eigen::MatrixXd pinv;
        Eigen::MatrixXd p_inv = p.completeOrthogonalDecomposition().pseudoInverse();

        pseudo_inverse(p,pinv);

        cout << " pseudo inv: " << endl << pinv << endl;
        


        err = command_q_d - q_curr;
        dot_err = command_dot_q_d - dot_q_curr;


        
        cout << " err"<< endl << err <<endl;
        

        //abort();

        kp1 = 350;
        kp2 = 350;
        kp3 = 350;
        kp4 = 350;

        kv1 = 10;

        Kp = Eigen::MatrixXd::Identity(18,18);
        Kv = kv1 * Eigen::MatrixXd::Identity(18,18);

        Kp(1,1) = kp1; 
        Kp(2,2) = kp1;
        Kp(3,3) = kp1;
        Kp(4,4) = kp1;
        Kp(5,5) = kp1;
        Kp(6,6) = kp1;
        Kp(7,7) = kp2;
        Kp(8,8) = kp3;
        Kp(9,9) = kp4;
        Kp(10,10) = kp2;
        Kp(11,11) = kp3;
        Kp(12,12) = kp4;
        Kp(13,13) = kp2;
        Kp(14,14) = kp3;
        Kp(15,15) = kp4;
        Kp(16,16) = kp2;
        Kp(17,17) = kp3;
        Kp(18,18) = kp4;
        

       
        
        tau_cmd = pinv * Nc_trans * (M_* (Kp*err + Kv*dot_err) + cor_ + g_);
        
                

        for (size_t i=0; i<12; i++)
        {
           joint_handle[i].setCommand(tau_cmd[i]);
        }
        
        cout << "coppie command"<< endl << tau_cmd <<endl;
        
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