#include <kdl/tree.hpp>

namespace kinematics_kdl {
void get(KDL::Tree & tree)
{
	{
		KDL::Joint joint("backLeftAimingHip", KDL::Vector(0,0.36,0.27), KDL::Vector(0,0,1), KDL::Joint::JointType(0));
		KDL::Frame frame(KDL::Rotation(4.89658e-12,1,0,-1,4.89658e-12,0,0,0,1), KDL::Vector(0,0.36,0.27));
		KDL::RigidBodyInertia inertia(0, KDL::Vector(0,0,0), KDL::RotationalInertia(0,0,0,0,0,0));
		KDL::Segment segment("backLeftHip", joint, frame, inertia);
		tree.addSegment(segment, tree.getRootSegment()->second.segment.getName());
	}
	{
		KDL::Joint joint("backLeftLiftingHip", KDL::Vector(-0.36,0.12,-0.12), KDL::Vector(0,1,0), KDL::Joint::JointType(0));
		KDL::Frame frame(KDL::Rotation(1,0,0,0,1,0,0,0,1), KDL::Vector(-0.36,0.12,-0.12));
		KDL::RigidBodyInertia inertia(0, KDL::Vector(0,0,0), KDL::RotationalInertia(0,0,0,0,0,0));
		KDL::Segment segment("backLeftThigh", joint, frame, inertia);
		tree.addSegment(segment, "backLeftHip");
	}
	{
		KDL::Joint joint("backLeftKnee", KDL::Vector(-0.36,0,0), KDL::Vector(0,-1,0), KDL::Joint::JointType(0));
		KDL::Frame frame(KDL::Rotation(1,0,0,0,1,0,0,0,1), KDL::Vector(-0.36,0,0));
		KDL::RigidBodyInertia inertia(0, KDL::Vector(0,0,0), KDL::RotationalInertia(0,0,0,0,0,0));
		KDL::Segment segment("backLeftCalf", joint, frame, inertia);
		tree.addSegment(segment, "backLeftThigh");
	}
	{
		KDL::Joint joint("backLeftAnkle", KDL::Joint::JointType(8));
		KDL::Frame frame(KDL::Rotation(1,0,0,0,1,0,0,0,1), KDL::Vector(-0.33,-0.12,0));
		KDL::RigidBodyInertia inertia(0, KDL::Vector(0,0,0), KDL::RotationalInertia(0,0,0,0,0,0));
		KDL::Segment segment("backLeftFoot", joint, frame, inertia);
		tree.addSegment(segment, "backLeftCalf");
	}
	{
		KDL::Joint joint("backRightAimingHip", KDL::Vector(-0.36,0,0.27), KDL::Vector(0,0,1), KDL::Joint::JointType(0));
		KDL::Frame frame(KDL::Rotation(1,0,0,0,1,0,0,0,1), KDL::Vector(-0.36,0,0.27));
		KDL::RigidBodyInertia inertia(0, KDL::Vector(0,0,0), KDL::RotationalInertia(0,0,0,0,0,0));
		KDL::Segment segment("backRightHip", joint, frame, inertia);
		tree.addSegment(segment, tree.getRootSegment()->second.segment.getName());
	}
	{
		KDL::Joint joint("backRightLiftingHip", KDL::Vector(-0.36,0.12,-0.12), KDL::Vector(0,1,0), KDL::Joint::JointType(0));
		KDL::Frame frame(KDL::Rotation(1,0,0,0,1,0,0,0,1), KDL::Vector(-0.36,0.12,-0.12));
		KDL::RigidBodyInertia inertia(0, KDL::Vector(0,0,0), KDL::RotationalInertia(0,0,0,0,0,0));
		KDL::Segment segment("backRightThigh", joint, frame, inertia);
		tree.addSegment(segment, "backRightHip");
	}
	{
		KDL::Joint joint("backRightKnee", KDL::Vector(-0.36,0,0), KDL::Vector(0,-1,0), KDL::Joint::JointType(0));
		KDL::Frame frame(KDL::Rotation(1,0,0,0,1,0,0,0,1), KDL::Vector(-0.36,0,0));
		KDL::RigidBodyInertia inertia(0, KDL::Vector(0,0,0), KDL::RotationalInertia(0,0,0,0,0,0));
		KDL::Segment segment("backRightCalf", joint, frame, inertia);
		tree.addSegment(segment, "backRightThigh");
	}
	{
		KDL::Joint joint("backRightAnkle", KDL::Joint::JointType(8));
		KDL::Frame frame(KDL::Rotation(1,0,0,0,1,0,0,0,1), KDL::Vector(-0.33,-0.12,0));
		KDL::RigidBodyInertia inertia(0, KDL::Vector(0,0,0), KDL::RotationalInertia(0,0,0,0,0,0));
		KDL::Segment segment("backRightFoot", joint, frame, inertia);
		tree.addSegment(segment, "backRightCalf");
	}
	{
		KDL::Joint joint("frontLeftAimingHip", KDL::Vector(0.36,0,0.27), KDL::Vector(0,0,1), KDL::Joint::JointType(0));
		KDL::Frame frame(KDL::Rotation(-1,2.06823e-13,0,-2.06823e-13,-1,0,0,0,1), KDL::Vector(0.36,0,0.27));
		KDL::RigidBodyInertia inertia(0, KDL::Vector(0,0,0), KDL::RotationalInertia(0,0,0,0,0,0));
		KDL::Segment segment("frontLeftHip", joint, frame, inertia);
		tree.addSegment(segment, tree.getRootSegment()->second.segment.getName());
	}
	{
		KDL::Joint joint("frontLeftLiftingHip", KDL::Vector(-0.36,0.12,-0.12), KDL::Vector(0,1,0), KDL::Joint::JointType(0));
		KDL::Frame frame(KDL::Rotation(1,0,0,0,1,0,0,0,1), KDL::Vector(-0.36,0.12,-0.12));
		KDL::RigidBodyInertia inertia(0, KDL::Vector(0,0,0), KDL::RotationalInertia(0,0,0,0,0,0));
		KDL::Segment segment("frontLeftThigh", joint, frame, inertia);
		tree.addSegment(segment, "frontLeftHip");
	}
	{
		KDL::Joint joint("frontLeftKnee", KDL::Vector(-0.36,0,0), KDL::Vector(0,-1,0), KDL::Joint::JointType(0));
		KDL::Frame frame(KDL::Rotation(1,0,0,0,1,0,0,0,1), KDL::Vector(-0.36,0,0));
		KDL::RigidBodyInertia inertia(0, KDL::Vector(0,0,0), KDL::RotationalInertia(0,0,0,0,0,0));
		KDL::Segment segment("frontLeftCalf", joint, frame, inertia);
		tree.addSegment(segment, "frontLeftThigh");
	}
	{
		KDL::Joint joint("frontLeftAnkle", KDL::Joint::JointType(8));
		KDL::Frame frame(KDL::Rotation(1,0,0,0,1,0,0,0,1), KDL::Vector(-0.33,-0.12,0));
		KDL::RigidBodyInertia inertia(0, KDL::Vector(0,0,0), KDL::RotationalInertia(0,0,0,0,0,0));
		KDL::Segment segment("frontLeftFoot", joint, frame, inertia);
		tree.addSegment(segment, "frontLeftCalf");
	}
	{
		KDL::Joint joint("frontRightAimingHip", KDL::Vector(0,-0.36,0.27), KDL::Vector(0,0,1), KDL::Joint::JointType(0));
		KDL::Frame frame(KDL::Rotation(4.89658e-12,-1,0,1,4.89658e-12,0,0,0,1), KDL::Vector(0,-0.36,0.27));
		KDL::RigidBodyInertia inertia(0, KDL::Vector(0,0,0), KDL::RotationalInertia(0,0,0,0,0,0));
		KDL::Segment segment("frontRightHip", joint, frame, inertia);
		tree.addSegment(segment, tree.getRootSegment()->second.segment.getName());
	}
	{
		KDL::Joint joint("frontRightLiftingHip", KDL::Vector(-0.36,0.12,-0.12), KDL::Vector(0,1,0), KDL::Joint::JointType(0));
		KDL::Frame frame(KDL::Rotation(1,0,0,0,1,0,0,0,1), KDL::Vector(-0.36,0.12,-0.12));
		KDL::RigidBodyInertia inertia(0, KDL::Vector(0,0,0), KDL::RotationalInertia(0,0,0,0,0,0));
		KDL::Segment segment("frontRightThigh", joint, frame, inertia);
		tree.addSegment(segment, "frontRightHip");
	}
	{
		KDL::Joint joint("frontRightKnee", KDL::Vector(-0.36,0,0), KDL::Vector(0,-1,0), KDL::Joint::JointType(0));
		KDL::Frame frame(KDL::Rotation(1,0,0,0,1,0,0,0,1), KDL::Vector(-0.36,0,0));
		KDL::RigidBodyInertia inertia(0, KDL::Vector(0,0,0), KDL::RotationalInertia(0,0,0,0,0,0));
		KDL::Segment segment("frontRightCalf", joint, frame, inertia);
		tree.addSegment(segment, "frontRightThigh");
	}
	{
		KDL::Joint joint("frontRightAnkle", KDL::Joint::JointType(8));
		KDL::Frame frame(KDL::Rotation(1,0,0,0,1,0,0,0,1), KDL::Vector(-0.33,-0.12,0));
		KDL::RigidBodyInertia inertia(0, KDL::Vector(0,0,0), KDL::RotationalInertia(0,0,0,0,0,0));
		KDL::Segment segment("frontRightFoot", joint, frame, inertia);
		tree.addSegment(segment, "frontRightCalf");
	}
}

}
