#include <iostream>

#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/Core>

void output_tree(KDL::TreeElement element, std::string targetName)
{
	for (auto & child : element.children) {
		std::cout << "\t{" << std::endl;

		KDL::Segment const & segment = child->second.segment;

		KDL::Joint const & joint = segment.getJoint();
		std::cout << "\t\tKDL::Joint joint(";
		std::cout << "\"" << joint.getName() << "\", ";
		if (joint.getType() == KDL::Joint::RotAxis || joint.getType() == KDL::Joint::TransAxis) {
			KDL::Vector origin = joint.JointOrigin();
			std::cout << "KDL::Vector(" << origin.x() << "," << origin.y() << "," << origin.z() << "), ";
			KDL::Vector axis = joint.JointAxis();
			std::cout << "KDL::Vector(" << axis.x() << "," << axis.y() << "," << axis.z() << "), ";
		}
		std::cout << "KDL::Joint::JointType(" << joint.getType() << ")";
		std::cout << ");" << std::endl;

		KDL::Frame frame = segment.getFrameToTip();
		std::cout << "\t\tKDL::Frame frame(";
		std::cout << "KDL::Rotation(";
		for (int i = 0; i < 9; ++ i)
			std::cout << frame.M.data[i] << (i < 8 ? "," : "");
		std::cout << "), ";
		std::cout << "KDL::Vector(" << frame.p.x() << "," << frame.p.y() << "," << frame.p.z() << ")";
		std::cout << ");" << std::endl;

		KDL::RigidBodyInertia const & inertia = segment.getInertia();
		std::cout << "\t\tKDL::RigidBodyInertia inertia(";
		std::cout << inertia.getMass() << ", ";
		KDL::Vector cog = inertia.getCOG();
		std::cout << "KDL::Vector(" << cog.x() << "," << cog.y() << "," << cog.z() << "), ";
		KDL::RotationalInertia adjusted = inertia.getRotationalInertia();
		Eigen::Vector3d eig = Eigen::Map<const Eigen::Vector3d>(cog.data);
		Eigen::Matrix3d ri = Eigen::Map<const Eigen::Matrix3d>(adjusted.data) + inertia.getMass() * (eig*eig.transpose() - eig.dot(eig) * Eigen::Matrix3d::Identity());
		std::cout << "KDL::RotationalInertia(" << ri(0,0) << "," << ri(1,1) << "," << ri(2,2) << "," << ri(1,0) << "," << ri(2,0) << "," << ri(2,1) << ")";
		std::cout << ");" << std::endl;

		std::cout << "\t\tKDL::Segment segment(";
		std::cout << "\"" << segment.getName() << "\", ";
		std::cout << "joint, ";
		std::cout << "frame, ";
		std::cout << "inertia";
		std::cout << ");" << std::endl;

		std::cout << "\t\ttree.addSegment(segment, " << targetName << ");" << std::endl;

		std::cout << "\t}" << std::endl;

		output_tree(child->second, "\"" + segment.getName() + "\"");
	}
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "urdf_to_kdl");
	if (argc != 3) {
		ROS_ERROR("Usage: urdf_to_kdl name file.urdf");
		return -1;
	}

	std::string function_name = argv[1];
	std::string urdf_file = argv[2];

	KDL::Tree tree;
	if (!kdl_parser::treeFromFile(urdf_file, tree)) {
		ROS_ERROR("Failed to construct kdl tree");
		return -1;
	}

	std::cout << "#include <kdl/tree.hpp>" << std::endl;
	std::cout << std::endl;
	std::cout << "void " << function_name << "(KDL::Tree & tree)" << std::endl;
	std::cout << "{" << std::endl;
	
	output_tree(tree.getRootSegment()->second, "tree.getRootSegment()->second.segment.getName()");

	std::cout << "}" << std::endl;

	return 0;
}
