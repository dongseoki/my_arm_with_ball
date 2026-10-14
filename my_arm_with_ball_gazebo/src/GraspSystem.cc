#include <chrono>
#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace my_arm_with_ball_gazebo
{
namespace sim = gz::sim;
namespace comp = gz::sim::components;

class GraspSystem : public sim::System,
    public sim::ISystemConfigure, public sim::ISystemPreUpdate,
    public sim::ISystemPostUpdate
{
public:
    ~GraspSystem() override
    {
        if (executor_) {
            executor_->cancel();
        }
        if (thread_.joinable()) {
            thread_.join();
        }
        if (context_) {
            context_->shutdown("Grasp plugin unloaded");
        }
    }

    void Configure(const sim::Entity &, const std::shared_ptr<const sdf::Element> &,
        sim::EntityComponentManager &, sim::EventManager &) override
    {
        context_ = std::make_shared<rclcpp::Context>();
        context_->init(0, nullptr);
        rclcpp::NodeOptions options;
        options.context(context_);
        node_ = std::make_shared<rclcpp::Node>("gazebo_grasp_system", options);
        contactPub_ = node_->create_publisher<std_msgs::msg::Bool>("/grasp/contact", 10);
        attachedPub_ = node_->create_publisher<std_msgs::msg::Bool>("/grasp/attached", 10);
        posePub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/grasp/ball_pose", 10);
        attachService_ = node_->create_service<std_srvs::srv::Trigger>("/grasp/attach",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                Request(Command::Attach, *response);
            });
        detachService_ = node_->create_service<std_srvs::srv::Trigger>("/grasp/detach",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                Request(Command::Detach, *response);
            });
        rclcpp::ExecutorOptions executorOptions;
        executorOptions.context = context_;
        executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executorOptions);
        executor_->add_node(node_);
        thread_ = std::thread([this]() {executor_->spin();});
    }

    void PreUpdate(const sim::UpdateInfo &info, sim::EntityComponentManager &ecm) override
    {
        if (info.paused) {
            return;
        }
        if (hand_ == sim::kNullEntity) {
            Discover(ecm);
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (!pending_) {
            return;
        }
        success_ = false;
        if (hand_ == sim::kNullEntity || ball_ == sim::kNullEntity) {
            message_ = "Robot or ball entities are not ready";
        } else if (command_ == Command::Attach && joint_ != sim::kNullEntity) {
            message_ = "Ball is already attached";
            success_ = true;
        } else if (command_ == Command::Attach) {
            if (CanAttach(ecm)) {
                Attach(ecm);
                message_ = "Ball attached";
                success_ = true;
            } else {
                message_ = "Requires finger-ball contact within 3 seconds and joint sum above 5 degrees";
            }
        } else {
            if (joint_ != sim::kNullEntity) {
                ecm.RequestRemoveEntity(joint_);
                joint_ = sim::kNullEntity;
            }
            message_ = "Ball detached";
            success_ = true;
        }
        pending_ = false;
        processed_ = true;
    }

    void PostUpdate(const sim::UpdateInfo &info,
        const sim::EntityComponentManager &ecm) override
    {
        if (info.paused) {
            return;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (processed_) {
                processed_ = false;
                completed_ = true;
                condition_.notify_all();
            }
        }
        if (ball_ == sim::kNullEntity) {
            return;
        }
        bool contact = false;
        for (const auto finger : {leftCollision_, rightCollision_}) {
            const auto *data = ecm.Component<comp::ContactSensorData>(finger);
            if (!data) {
                continue;
            }
            for (const auto &entry : data->Data().contact()) {
                if ((entry.collision1().id() == finger && entry.collision2().id() == ballCollision_) ||
                    (entry.collision2().id() == finger && entry.collision1().id() == ballCollision_)) {
                    contact = true;
                }
            }
        }
        bool attached;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (contact && JointOpeningValid(ecm)) {
                validContactTime_ = std::chrono::steady_clock::now();
            }
            attached = joint_ != sim::kNullEntity;
        }
        if (info.iterations % 20 != 0) {
            return;
        }
        std_msgs::msg::Bool state;
        state.data = contact;
        contactPub_->publish(state);
        state.data = attached;
        attachedPub_->publish(state);
        const auto pose = sim::worldPose(ball_, ecm);
        geometry_msgs::msg::PoseStamped message;
        message.header.frame_id = "world";
        message.header.stamp = rclcpp::Time(
            std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count());
        message.pose.position.x = pose.Pos().X();
        message.pose.position.y = pose.Pos().Y();
        message.pose.position.z = pose.Pos().Z();
        message.pose.orientation.x = pose.Rot().X();
        message.pose.orientation.y = pose.Rot().Y();
        message.pose.orientation.z = pose.Rot().Z();
        message.pose.orientation.w = pose.Rot().W();
        posePub_->publish(message);
    }

private:
    enum class Command {Attach, Detach};

    bool JointOpeningValid(const sim::EntityComponentManager &ecm) const
    {
        const auto *left = ecm.Component<comp::JointPosition>(leftJoint_);
        const auto *right = ecm.Component<comp::JointPosition>(rightJoint_);
        return left && right && !left->Data().empty() && !right->Data().empty() &&
            std::isfinite(left->Data()[0]) && std::isfinite(right->Data()[0]) &&
            left->Data()[0] + right->Data()[0] > 5.0 * std::acos(-1.0) / 180.0;
    }

    bool CanAttach(const sim::EntityComponentManager &ecm) const
    {
        // The controller may relax contact while its cancellation completes.
        // Keep a short, qualified contact proof for the explicit attach request.
        return hand_ != sim::kNullEntity && ball_ != sim::kNullEntity &&
            JointOpeningValid(ecm) &&
            std::chrono::steady_clock::now() - validContactTime_ < std::chrono::seconds(3);
    }

    void Attach(sim::EntityComponentManager &ecm)
    {
        joint_ = ecm.CreateEntity();
        ecm.CreateComponent(joint_, comp::DetachableJoint({hand_, ball_, "fixed"}));
        validContactTime_ = std::chrono::steady_clock::time_point();
    }

    void Discover(sim::EntityComponentManager &ecm)
    {
        const auto robot = ecm.EntityByComponents(comp::Model(), comp::Name("ur5_rg2"));
        const auto apple = ecm.EntityByComponents(comp::Model(), comp::Name("apple"));
        const auto hand = ecm.EntityByComponents(comp::Link(), comp::Name("rg2_hand"), comp::ParentEntity(robot));
        const auto ball = ecm.EntityByComponents(comp::Link(), comp::Name("ball"), comp::ParentEntity(apple));
        const auto leftLink = ecm.EntityByComponents(comp::Link(), comp::Name("rg2_leftfinger"), comp::ParentEntity(robot));
        const auto rightLink = ecm.EntityByComponents(comp::Link(), comp::Name("rg2_rightfinger"), comp::ParentEntity(robot));
        const auto left = ecm.EntityByComponents(comp::Collision(), comp::ParentEntity(leftLink));
        const auto right = ecm.EntityByComponents(comp::Collision(), comp::ParentEntity(rightLink));
        const auto ballCollision = ecm.EntityByComponents(comp::Collision(), comp::ParentEntity(ball));
        const auto leftJoint = ecm.EntityByComponents(comp::Name("rg2_finger_joint1"), comp::ParentEntity(robot));
        const auto rightJoint = ecm.EntityByComponents(comp::Name("rg2_finger_joint2"), comp::ParentEntity(robot));
        if (hand == sim::kNullEntity || ball == sim::kNullEntity || left == sim::kNullEntity ||
            right == sim::kNullEntity || ballCollision == sim::kNullEntity ||
            leftJoint == sim::kNullEntity || rightJoint == sim::kNullEntity) {
            return;
        }
        hand_ = hand;
        ball_ = ball;
        leftCollision_ = left;
        rightCollision_ = right;
        ballCollision_ = ballCollision;
        leftJoint_ = leftJoint;
        rightJoint_ = rightJoint;
        ecm.CreateComponent(left, comp::ContactSensorData());
        ecm.CreateComponent(right, comp::ContactSensorData());
        if (!ecm.Component<comp::JointPosition>(leftJoint)) {
            ecm.CreateComponent(leftJoint, comp::JointPosition());
        }
        if (!ecm.Component<comp::JointPosition>(rightJoint)) {
            ecm.CreateComponent(rightJoint, comp::JointPosition());
        }
    }

    void Request(Command command, std_srvs::srv::Trigger::Response &response)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        command_ = command;
        pending_ = true;
        completed_ = false;
        processed_ = false;
        if (!condition_.wait_for(lock, std::chrono::seconds(2), [this]() {return completed_;})) {
            pending_ = false;
            response.success = false;
            response.message = "Simulation did not process request (paused or stopped)";
            return;
        }
        response.success = success_;
        response.message = message_;
    }

    sim::Entity hand_{sim::kNullEntity}, ball_{sim::kNullEntity};
    sim::Entity leftCollision_{sim::kNullEntity}, rightCollision_{sim::kNullEntity};
    sim::Entity ballCollision_{sim::kNullEntity}, leftJoint_{sim::kNullEntity};
    sim::Entity rightJoint_{sim::kNullEntity}, joint_{sim::kNullEntity};
    std::mutex mutex_;
    std::condition_variable condition_;
    bool processed_{false};
    bool pending_{false}, completed_{false}, success_{false};
    Command command_{Command::Detach};
    std::chrono::steady_clock::time_point validContactTime_;
    std::string message_;
    std::shared_ptr<rclcpp::Context> context_;
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread thread_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr contactPub_, attachedPub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr attachService_, detachService_;
};
}  // namespace my_arm_with_ball_gazebo

GZ_ADD_PLUGIN(my_arm_with_ball_gazebo::GraspSystem, gz::sim::System,
    my_arm_with_ball_gazebo::GraspSystem::ISystemConfigure,
    my_arm_with_ball_gazebo::GraspSystem::ISystemPreUpdate,
    my_arm_with_ball_gazebo::GraspSystem::ISystemPostUpdate)
