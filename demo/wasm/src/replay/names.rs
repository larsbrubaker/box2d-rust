//! Display names for the Replay inspector (sample_replay.cpp Replay*TypeName).

use box2d_rust::collision::ShapeType;
use box2d_rust::joint::JointType;
use box2d_rust::recording::RecQueryType;
use box2d_rust::types::BodyType;

pub fn body_type_name(t: BodyType) -> &'static str {
    match t {
        BodyType::Static => "static",
        BodyType::Kinematic => "kinematic",
        BodyType::Dynamic => "dynamic",
    }
}

pub fn shape_type_name(t: ShapeType) -> &'static str {
    match t {
        ShapeType::Circle => "circle",
        ShapeType::Capsule => "capsule",
        ShapeType::Segment => "segment",
        ShapeType::Polygon => "polygon",
        ShapeType::ChainSegment => "chain segment",
    }
}

pub fn joint_type_name(t: JointType) -> &'static str {
    match t {
        JointType::Distance => "distance",
        JointType::Filter => "filter",
        JointType::Motor => "motor",
        JointType::Prismatic => "prismatic",
        JointType::Revolute => "revolute",
        JointType::Weld => "weld",
        JointType::Wheel => "wheel",
    }
}

pub fn query_type_name(t: RecQueryType) -> &'static str {
    match t {
        RecQueryType::OverlapAabb => "overlap AABB",
        RecQueryType::OverlapShape => "overlap shape",
        RecQueryType::CastRay => "cast ray",
        RecQueryType::CastShape => "cast shape",
        RecQueryType::CollideMover => "collide mover",
        RecQueryType::CastRayClosest => "cast ray closest",
        RecQueryType::CastMover => "cast mover",
        RecQueryType::ShapeTestPoint => "shape test point",
        RecQueryType::ShapeRayCast => "shape ray cast",
    }
}
