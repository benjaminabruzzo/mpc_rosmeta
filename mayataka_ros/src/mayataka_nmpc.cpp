#include <mayataka_nmpc.hpp>
class mayataka_nmpc
{
	private:		
		/*--------- ROS Communication Containers ------------- */
		ros::NodeHandle n;
			ros::Subscriber NodeShutDown_sub;
			std::string s_shutdown_topic;


		int dim_state_, dim_control_input_, dim_constraints_;

		// Define the model in NMPC.
		NMPCModel nmpc_model;


	public:
		mayataka_nmpc()
		{
			if(ros::param::get("~shutdown_topic",s_shutdown_topic)){} else {s_shutdown_topic = "/kill";}

			ROS_INFO("mayataka_nmpc:: NodeShutDown_sub s_shutdown_topic.");
			NodeShutDown_sub 	= n.subscribe(s_shutdown_topic,		1, &mayataka_nmpc::nodeShutDown, 	this);


			// Define the solver of C/GMRES.
			// ContinuationGMRES cgmres_solver(nmpc_model, 1.0, 1.0, 50, 1.0e-06, 1000, 5);
			MultipleShootingCGMRES cgmres_solver(nmpc_model, 1.0, 1.0, 50, 1.0e-06, 1000, 3);

			// Define the simulator.
			Simulator cgmres_simulator(nmpc_model);

			// Set the initial state.
			Eigen::VectorXd initial_state(nmpc_model.dimState());
			initial_state = Eigen::VectorXd::Zero(nmpc_model.dimState());

			// Set the initial guess of the control input vector.
			Eigen::VectorXd initial_guess_control_input(nmpc_model.dimControlInput()+nmpc_model.dimConstraints());
			initial_guess_control_input = Eigen::VectorXd::Zero(nmpc_model.dimControlInput()+nmpc_model.dimConstraints());


			ROS_INFO("mayataka_nmpc:: mayataka_nmpc started.");
			// Initialize the solution of the C/GMRES method.
			cgmres_solver.initSolution(0, initial_state, initial_guess_control_input, 1.0e-06, 50);

			// Perform a numerical simulation.
			cgmres_simulator.simulation(cgmres_solver, initial_state, 0, 10, 0.001, "example");


		}

		void nodeShutDown(const std_msgs::EmptyConstPtr& msg)
		{
			ROS_INFO("mayataka_nmpc:: Shutdown requested..");
			ros::Duration(1.5).sleep();
			ros::shutdown();
		}
};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "mayataka_nmpc");
	mayataka_nmpc model_;
	ros::spin();
	return 0;
}

