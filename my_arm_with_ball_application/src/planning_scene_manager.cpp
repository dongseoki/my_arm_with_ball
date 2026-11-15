#include "planning_scene_manager.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace pick_place
{
namespace
{
const std::vector<std::string> TOUCH = {"rg2_hand", "rg2_leftfinger", "rg2_rightfinger"};
}

PlanningSceneManager::PlanningSceneManager(GetScene get_scene, ApplyScene apply_scene)
    : get_scene_(std::move(get_scene)), apply_scene_(std::move(apply_scene))
{
}

/**
 * @brief MoveIt에 로봇 상태와 변환 등을 포함한 현재 Planning Scene을 요청한다.
 * @return 서비스에서 수신한 Planning Scene 사본.
 * @throws std::runtime_error 장면 조회 서비스를 사용할 수 없거나 응답 대기에 실패한 경우.
 */
moveit_msgs::msg::PlanningScene PlanningSceneManager::scene()
{
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components = 1023;
    // Complete live scene, including robot state and transforms.
    return get_scene_(request)->scene;
}

/**
 * @brief Planning Scene 변경 내용을 차이 메시지로 MoveIt에 적용한다.
 * @param diff 적용할 변경 내용. 장면과 로봇 상태의 is_diff를 true로 설정해 전송한다.
 * @throws std::runtime_error 서비스 호출에 실패하거나 MoveIt이 변경 내용을 거부한 경우.
 */
void PlanningSceneManager::apply(moveit_msgs::msg::PlanningScene diff)
{
    diff.is_diff = true;
    diff.robot_state.is_diff = true;
    auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    request->scene = std::move(diff);
    if (!apply_scene_(request)->success)
    {
        throw std::runtime_error("Planning Scene rejected diff");
    }
}

/**
 * @brief 반지름 0.0325 m인 공의 충돌 물체 메시지를 만든다.
 * @param pose frame 좌표계 기준 공의 중심 위치와 방향.
 * @param frame 공의 pose를 표현한 좌표계 이름.
 * @return ID가 apple이고 작업이 ADD인 구형 충돌 물체. 장면에는 아직 적용하지 않는다.
 */
moveit_msgs::msg::CollisionObject PlanningSceneManager::sphere(const geometry_msgs::msg::Pose &pose, const std::string &frame)
{
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = frame;
    object.id = "apple";
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    shape_msgs::msg::SolidPrimitive shape;
    shape.type = shape.SPHERE;
    shape.dimensions = {.0325};
    object.primitives.push_back(shape);
    object.primitive_poses.push_back(pose);
    return object;
}

/**
 * @brief MoveIt Planning Scene에서 공과 TOUCH에 지정된 링크 사이의 충돌 허용 여부를 설정한다.
 * @param allowed true이면 공과 해당 링크의 접촉을 허용하고, false이면 충돌 검사 대상으로 되돌린다.
 * @details 현재 Allowed Collision Matrix를 조회하고 필요한 항목을 추가한 뒤 변경 내용을 적용한다.
 *          공(apple)과 바닥(pick_area_floor) 사이의 충돌은 allowed 값과 관계없이 항상 허용한다.
 *          MoveIt의 충돌 검사 규칙만 변경하며 Gazebo의 물리적 접촉이나 부착 상태는 변경하지 않는다.
 * @throws std::runtime_error Planning Scene 조회 또는 변경 적용에 실패한 경우.
 */
void PlanningSceneManager::setHandBallCollisionAllowed(bool allowed)
{
    auto matrix = scene().allowed_collision_matrix;
    auto names = TOUCH;
    names.push_back("apple");
    names.push_back("pick_area_floor");
    for (const auto &name : names)
    {
        if (std::find(matrix.entry_names.begin(), matrix.entry_names.end(), name) == matrix.entry_names.end())
        {
            matrix.entry_names.push_back(name);
            for (auto &entry : matrix.entry_values)
            {
                entry.enabled.push_back(false);
            }
            moveit_msgs::msg::AllowedCollisionEntry entry;
            entry.enabled.resize(matrix.entry_names.size(), false);
            matrix.entry_values.push_back(entry);
        }
    }
    auto index = [&matrix](const std::string &name)
    {
        return std::distance(matrix.entry_names.begin(), std::find(matrix.entry_names.begin(), matrix.entry_names.end(), name));
    };
    const auto apple = index("apple");
    for (const auto &name : TOUCH)
    {
        matrix.entry_values[apple].enabled[index(name)] = allowed;
        matrix.entry_values[index(name)].enabled[apple] = allowed;
    }
    matrix.entry_values[apple].enabled[index("pick_area_floor")] = true;
    matrix.entry_values[index("pick_area_floor")].enabled[apple] = true;
    moveit_msgs::msg::PlanningScene diff;
    diff.allowed_collision_matrix = matrix;
    apply(diff);
}

/**
 * @brief Planning Scene의 공이 기대한 소속·형상·좌표계·위치로 등록됐는지 검증한다.
 * @param attachment Attached이면 rg2_hand에 부착된 공 하나만, Detached이면 world 물체 목록의
 *                 공 하나만 존재해야 한다.
 * @param expected 비교 기준 공. header.frame_id와 primitive_poses[0]의 중심 위치를 사용한다.
 * @details 장면 조회 → 공 수집 → 소속·개수 확인 → 형상·좌표계 확인 → 중심 위치 계산
 *          → 위치 오차 확인의 6단계로 처리한다. 반지름은 0.0325 m를 기준으로 검사한다.
 *          장면을 변경하거나 Gazebo의 실제 파지 상태를 확인하지는 않는다.
 *          구의 중심 위치만 비교하며 공의 방향 일치 여부는 검사하지 않는다.
 * @throws std::runtime_error 장면 조회에 실패하거나 공의 소속·개수·형상·좌표계가
 *                            맞지 않거나 중심 위치 오차가 0.005 m를 초과한 경우.
 */
void PlanningSceneManager::verifyScene(AttachmentState attachment, const moveit_msgs::msg::CollisionObject &expected)
{
    const bool attached = attachment == AttachmentState::Attached;
    // 1. MoveIt에서 현재 Planning Scene을 조회한다.
    const auto current = scene();
    // 2. 양쪽 목록의 apple 개수를 세고, 기대한 소속과 부착 링크에 맞는 공을 수집한다.
    std::vector<moveit_msgs::msg::CollisionObject> objects;
    size_t world_count = 0, attached_count = 0;
    for (const auto &obj : current.world.collision_objects)
    {
        if (obj.id == "apple")
        {
            ++world_count;
            if (!attached)
            {
                objects.push_back(obj);
            }
        }
    }
    for (const auto &obj : current.robot_state.attached_collision_objects)
    {
        if (obj.object.id == "apple")
        {
            ++attached_count;
            if (attached && obj.link_name == "rg2_hand")
            {
                objects.push_back(obj.object);
            }
        }
    }
    // 3. 원하는 목록에 공이 정확히 하나 있고 반대쪽에는 없으며, 검증 후보도 하나인지 확인한다.
    if (world_count != (attached ? 0u : 1u) || attached_count != (attached ? 1u : 0u) || objects.size() != 1)
    {
        throw std::runtime_error("Planning Scene world/attached membership mismatch");
    }
    // 4. 단일 구 형상, 반지름 0.0325 m(허용 오차 1e-6 m), 기대한 좌표계인지 확인한다.
    const auto &obj = objects.front();
    if (obj.primitives.size() != 1 || obj.primitive_poses.size() != 1 ||
        obj.primitives[0].type != shape_msgs::msg::SolidPrimitive::SPHERE ||
        obj.primitives[0].dimensions.size() != 1 || std::abs(obj.primitives[0].dimensions[0] - .0325) > 1e-6 ||
        obj.header.frame_id != expected.header.frame_id)
    {
        throw std::runtime_error("Planning Scene sphere geometry mismatch");
    }
    // 5. 물체 원점의 위치·회전과 구의 로컬 중심을 합성해 header.frame_id 기준 중심을 계산한다.
    // MoveIt이 primitive_poses를 물체 원점 쪽으로 재배치할 수 있어 로컬 위치만 비교하지 않는다.
    const auto &origin = obj.pose;
    Eigen::Quaterniond rotation(origin.orientation.w, origin.orientation.x,
                                origin.orientation.y, origin.orientation.z);
    if (rotation.norm() < 1e-9)
    {
        rotation = Eigen::Quaterniond::Identity();
    }
    else
    {
        rotation.normalize();
    }
    const auto &local = obj.primitive_poses[0].position;
    const Eigen::Vector3d actual = Eigen::Vector3d(origin.position.x, origin.position.y,
                                                   origin.position.z) +
                                   rotation * Eigen::Vector3d(local.x, local.y, local.z);
    // 6. 기대한 중심과의 거리 오차가 0.005 m(5 mm)를 초과하면 검증 실패로 처리한다.
    const auto &b = expected.primitive_poses[0].position;
    if ((actual - Eigen::Vector3d(b.x, b.y, b.z)).norm() > .005)
    {
        throw std::runtime_error("Planning Scene ball pose mismatch: actual=" +
                                 std::to_string(actual.x()) + "," + std::to_string(actual.y()) + "," +
                                 std::to_string(actual.z()) + " expected=" + std::to_string(b.x) + "," +
                                 std::to_string(b.y) + "," + std::to_string(b.z));
    }
}

/**
 * @brief Gazebo에서 측정한 공의 위치·자세를 MoveIt Planning Scene에 반영하고 검증한다.
 * @param update Attach는 손에 부착, Detach는 부착 정보 제거 후 world 등록,
 *               RefreshWorld는 부착 정보 제거 없이 world 물체 갱신을 수행한다.
 * @details 측정 → 부착 여부에 따른 변경 내용 구성 → 적용 → 검증의 4단계로 처리한다.
 *          Gazebo의 실제 부착·분리는 수행하지 않고 MoveIt의 충돌 물체 정보만 동기화한다.
 * @throws std::runtime_error 측정값 또는 공의 자세가 유효하지 않거나,
 *                            Planning Scene 적용·검증에 실패한 경우.
 */
void PlanningSceneManager::syncScene(SceneUpdate update,
    const geometry_msgs::msg::PoseStamped &measured_ball,
    const std::function<Eigen::Isometry3d()> &hand_transform)
{
    const bool attached = update == SceneUpdate::Attach;
    // 1. 공의 현재 위치·자세를 읽고 변경 메시지와 검증 기준을 준비한다.
    moveit_msgs::msg::PlanningScene diff;
    moveit_msgs::msg::CollisionObject expected;
    // 2. 부착 여부에 맞게 Planning Scene에 반영할 변경 내용을 구성한다.
    if (attached)
    {
        // 현재 관절값으로 손의 world 기준 위치·자세를 계산한다.
        const auto hand = hand_transform();
        const auto &p = measured_ball.pose;
        Eigen::Quaterniond orientation(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
        if (!orientation.coeffs().allFinite() || orientation.norm() < 1e-9)
        {
            throw std::runtime_error("Invalid ball quaternion");
        }
        Eigen::Isometry3d world_ball = Eigen::Isometry3d::Identity();
        world_ball.linear() = orientation.normalized().toRotationMatrix();
        world_ball.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
        // world 기준 공의 위치·자세를 rg2_hand 기준으로 변환한다.
        const auto relative = hand.inverse() * world_ball;
        geometry_msgs::msg::Pose local;
        local.position.x = relative.translation().x();
        local.position.y = relative.translation().y();
        local.position.z = relative.translation().z();
        Eigen::Quaterniond q(relative.rotation());
        local.orientation.x = q.x();
        local.orientation.y = q.y();
        local.orientation.z = q.z();
        local.orientation.w = q.w();
        // 변환한 공을 손에 부착된 충돌 물체로 등록한다.
        expected = sphere(local, "rg2_hand");
        moveit_msgs::msg::AttachedCollisionObject object;
        object.link_name = "rg2_hand";
        object.touch_links = TOUCH;
        object.object = expected;
        diff.robot_state.attached_collision_objects.push_back(object);
    }
    else
    {
        // 일반 분리에서는 부착 정보를 제거하고, 단순 갱신에서는 제거를 생략한다.
        if (update == SceneUpdate::Detach)
        {
            moveit_msgs::msg::AttachedCollisionObject object;
            object.link_name = "rg2_hand";
            object.object.id = "apple";
            object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
            diff.robot_state.attached_collision_objects.push_back(object);
        }
        // 측정한 공을 world 기준의 독립 충돌 물체로 등록한다.
        expected = sphere(measured_ball.pose, "world");
        diff.world.collision_objects.push_back(expected);
    }
    // 3. 준비한 변경 내용을 MoveIt Planning Scene에 적용한다.
    apply(diff);
    // 4. 공의 world/attached 소속·개수, 구 형상·반지름, 좌표계·위치를 검증한다.
    verifyScene(attached ? AttachmentState::Attached : AttachmentState::Detached, expected);
}
}  // namespace pick_place
