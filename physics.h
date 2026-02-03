#ifndef __PHYSICS_H_
#define __PHYSICS_H_
#include <Ogre.h>
#include <OgreSingleton.h>
#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/EActivation.h>
void physics();
namespace JPH
{
class CharacterBase;
class ContactManifold;
class ContactSettings;
class SubShapeIDPair;
}
// Layer that objects can be in, determines which other objects it can collide with
// Typically you at least want to have 1 layer for moving bodies and 1 layer for static bodies, but you can have more
// layers if you want. E.g. you could have a layer for high detail collision (which is not used by the physics simulation
// but only if you do collision testing).
namespace Layers
{
static constexpr JPH::ObjectLayer NON_MOVING = 0;
static constexpr JPH::ObjectLayer MOVING = 1;
static constexpr JPH::ObjectLayer SENSORS = 2;
static constexpr JPH::ObjectLayer NUM_LAYERS = 3;
};

// Each broadphase layer results in a separate bounding volume tree in the broad phase. You at least want to have
// a layer for non-moving and moving objects to avoid having to update a tree full of static objects every frame.
// You can have a 1-on-1 mapping between object layers and broadphase layers (like in this case) but if you have
// many object layers you'll be creating many broad phase trees, which is not efficient. If you want to fine tune
// your broadphase layers define JPH_TRACK_BROADPHASE_STATS and look at the stats reported on the TTY.
namespace BroadPhaseLayers
{
static constexpr JPH::BroadPhaseLayer NON_MOVING(0);
static constexpr JPH::BroadPhaseLayer MOVING(1);
static constexpr uint NUM_LAYERS(2);
};

namespace JoltPhysics
{
template<class T>
Ogre::Vector3 convert(const T &vec)
{
    return {vec[0], vec[1], vec[2]};
}
template<class T> T convert(const Ogre::Vector3 &vec)
{
    return { vec.x, vec.y, vec.z };
}
Ogre::Quaternion convert(const JPH::QuatArg &rot);
JPH::Quat convert(const Ogre::Quaternion &rot);
struct ShapeData;
struct CompoundShapeBuilder {
	JPH::StaticCompoundShapeSettings shapeSettings;
	void addShape(JPH::ShapeRefC shape, const Ogre::Vector3 &position,
		      const Ogre::Quaternion &rotation);
	JPH::ShapeRefC build();
};
class ContactListener : public JPH::ContactListener {
public:
	struct ContactReport {
		bool entered;
		JPH::BodyID id1, id2;
		JPH::ContactManifold manifold;
		JPH::ContactSettings settings;
	};

private:
	std::list<ContactReport> reports;
	std::function<void(const ContactReport &report)> dispatch;
	std::map<JPH::BodyID, std::function<void(const ContactReport &report)> >
		listeners;

public:
	ContactListener();
	JPH::ValidateResult OnContactValidate(
		const JPH::Body &inBody1, const JPH::Body &inBody2,
		JPH::RVec3Arg inBaseOffset,
		const JPH::CollideShapeResult &inCollisionResult) override;
	void OnContactAdded(const JPH::Body &inBody1, const JPH::Body &inBody2,
			    const JPH::ContactManifold &inManifold,
			    JPH::ContactSettings &ioSettings) override;
	void OnContactPersisted(const JPH::Body &inBody1,
				const JPH::Body &inBody2,
				const JPH::ContactManifold &inManifold,
				JPH::ContactSettings &ioSettings) override;
	void
	OnContactRemoved(const JPH::SubShapeIDPair &inSubShapePair) override;
	void setDispatch(const std::function<void(const ContactReport &report)>
				 dispatcher)
	{
		dispatch = dispatcher;
	}
	void addListener(
		const JPH::BodyID &id,
		const std::function<void(const ContactReport &report)> listener)
	{
		listeners[id] = listener;
	}
	void removeListener(const JPH::BodyID &id)
	{
		listeners.erase(id);
	}
	void update();
};
}

class JoltPhysicsWrapper : public Ogre::Singleton<JoltPhysicsWrapper> {
public:
	JoltPhysics::ContactListener contacts;
	JoltPhysicsWrapper(Ogre::SceneManager *scnMgr,
			   Ogre::SceneNode *cameraNode);
	~JoltPhysicsWrapper();
	void update(float dt);
	void addBody(const JPH::BodyID &body, JPH::EActivation activation);
	bool isAdded(const JPH::BodyID &body);
	JPH::ShapeRefC createBoxShape(const Ogre::Vector3 &extents);
	JPH::ShapeRefC createSphereShape(float radius);
	JPH::ShapeRefC createCylinderShape(float halfHeight, float radius);
	JPH::ShapeRefC createMeshShape(Ogre::MeshPtr mesh);
	JPH::ShapeRefC createMeshShape(Ogre::String meshName);
	JPH::ShapeRefC createConvexHullShape(Ogre::MeshPtr mesh);
	JPH::ShapeRefC createConvexHullShape(Ogre::String meshName);
	JPH::ShapeRefC createHeightfieldShape(const float *samples,
					      Ogre::Vector3 offset,
					      Ogre::Vector3 scale,
					      int sampleCount);
	JPH::ShapeRefC createMutableCompoundShape(
		const std::vector<JPH::ShapeRefC> &shapes,
		const std::vector<Ogre::Vector3> &positions,
		const std::vector<Ogre::Quaternion> &rotations);
	JPH::ShapeRefC createStaticCompoundShape(
		const std::vector<JPH::ShapeRefC> &shapes,
		const std::vector<Ogre::Vector3> &positions,
		const std::vector<Ogre::Quaternion> &rotations);
	JPH::ShapeRefC
	createOffsetCenterOfMassShape(const Ogre::Vector3 &offset,
				      JPH::ShapeRefC shape);
    JPH::ShapeRefC
    createRotatedTranslatedShape(const Ogre::Vector3 &offset, const Ogre::Quaternion rotation,
                      JPH::ShapeRefC shape);
    JPH::BodyID createBody(const JPH::BodyCreationSettings &settings);
	JPH::BodyID createBody(const JPH::Shape *shape, float mass,
			       const Ogre::Vector3 &position,
			       const Ogre::Quaternion &rotation,
			       JPH::EMotionType motion, JPH::ObjectLayer layer);
	JPH::BodyID createBody(const JPH::Shape *shape, float mass,
			       Ogre::SceneNode *node, JPH::EMotionType motion,
			       JPH::ObjectLayer layer);
	JPH::BodyID createSensor(const JPH::Shape *shape,
				 const Ogre::Vector3 &position,
				 const Ogre::Quaternion &rotation,
				 JPH::EMotionType motion,
				 JPH::ObjectLayer layer);
	JPH::BodyID createSensor(const JPH::Shape *shape, Ogre::SceneNode *node,
				 JPH::EMotionType motion,
				 JPH::ObjectLayer layer);
	JPH::CharacterBase *createCharacter(Ogre::SceneNode *node,
					    float characterHeight,
					    float characterRadius);
	void addShapeToCompound(JPH::Ref<JPH::Shape> compoundShape,
				JPH::ShapeRefC childShape,
				const Ogre::Vector3 &position,
				const Ogre::Quaternion &rotation);
	void removeBody(const JPH::BodyID &id);
	void destroyBody(const JPH::BodyID &id);
	void setDebugDraw(bool enable);
	void broadphaseQuery(float dt, const Ogre::Vector3 &position,
			     std::set<JPH::BodyID> &inWater);
	void applyBuoyancyImpulse(JPH::BodyID id,
				  const Ogre::Vector3 &surfacePosition,
				  const Ogre::Vector3 &surfaceNormal,
				  float buoyancy, float linearDrag,
				  float angularDrag,
				  const Ogre::Vector3 &fluidVelocity,
				  const Ogre::Vector3 &gravity, float dt);
	void applyBuoyancyImpulse(JPH::BodyID id,
				  const Ogre::Vector3 &surfacePosition,
				  const Ogre::Vector3 &surfaceNormal,
				  float buoyancy, float linearDrag,
				  float angularDrag,
				  const Ogre::Vector3 &fluidVelocity, float dt);
	bool isActive(JPH::BodyID id);
	void activate(JPH::BodyID id);
	Ogre::Vector3 getPosition(JPH::BodyID id);
	void setPosition(JPH::BodyID id, const Ogre::Vector3 &position,
			 bool activate = true);
	Ogre::Quaternion getRotation(JPH::BodyID id);
	void setRotation(JPH::BodyID id, const Ogre::Quaternion &rotation,
			 bool activate = true);
	void getPositionAndRotation(JPH::BodyID id, Ogre::Vector3 &position,
				    Ogre::Quaternion &rotation);
	void setPositionAndRotation(JPH::BodyID id,
				    const Ogre::Vector3 &position,
				    const Ogre::Quaternion &rotation,
				    bool activate = true);
	Ogre::Vector3 getLinearVelocity(JPH::BodyID id);
	Ogre::Vector3 getAngularVelocity(JPH::BodyID id);
	float getFriction(JPH::BodyID id);
	void setFriction(JPH::BodyID id, float friction);
	void addAngularImpulse(const JPH::BodyID &id,
			       const Ogre::Vector3 &impulse);
	void setDispatch(std::function<void(const JoltPhysics::ContactListener::
						    ContactReport &report)>
				 dispatcher);
	void addContactListener(
		const JPH::BodyID &id,
		const std::function<void(const JoltPhysics::ContactListener::
						 ContactReport &report)>
			listener);
	void removeContactListener(const JPH::BodyID &id);
	bool raycastQuery(Ogre::Vector3 startPoint, Ogre::Vector3 endPoint,
              Ogre::Vector3 &position, JPH::BodyID &id);
};
#endif
