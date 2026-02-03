// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2025 Jorrit Rouwe
// SPDX-License-Identifier: CC0-1.0
// This file is in the public domain. It serves as an example to start building your own application using Jolt Physics. Feel free to copy paste without attribution!

// The Jolt headers don't include Jolt.h. Always include Jolt.h before including any other Jolt header.
// You can use Jolt.h in your precompiled header to speed up compilation.
#include <Ogre.h>
#include <OgreHardwareBufferManager.h>
#include <Jolt/Jolt.h>

// Jolt includes
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Character/Character.h>
#include <Jolt/Physics/Character/CharacterID.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>
#include <Jolt/Renderer/DebugRendererSimple.h>

// STL includes
#include <iostream>
#include <cstdarg>
#include <thread>
#include "physics.h"

// Disable common warnings triggered by Jolt, you can use JPH_SUPPRESS_WARNING_PUSH / JPH_SUPPRESS_WARNING_POP to store and restore the warning state
JPH_SUPPRESS_WARNINGS

// All Jolt symbols are in the JPH namespace

// Callback for traces, connect this to your own trace function if you have one
static void TraceImpl(const char *inFMT, ...)
{
	// Format the message
	va_list list;
	va_start(list, inFMT);
	char buffer[1024];
	vsnprintf(buffer, sizeof(buffer), inFMT, list);
	va_end(list);

	// Print to the TTY
	std::cout << buffer << std::endl;
}

#ifdef JPH_ENABLE_ASSERTS

// Callback for asserts, connect this to your own assert handler if you have one
static bool AssertFailedImpl(const char *inExpression, const char *inMessage,
			     const char *inFile, uint inLine)
{
	// Print to the TTY
	std::cout << inFile << ":" << inLine << ": (" << inExpression << ") "
		  << (inMessage != nullptr ? inMessage : "") << std::endl;

	// Breakpoint
	return true;
};

#endif // JPH_ENABLE_ASSERTS

/// Class that determines if two object layers can collide
class ObjectLayerPairFilterImpl : public JPH::ObjectLayerPairFilter {
public:
	virtual bool ShouldCollide(JPH::ObjectLayer inObject1,
				   JPH::ObjectLayer inObject2) const override
	{
		switch (inObject1) {
		case Layers::NON_MOVING:
			return inObject2 ==
			       Layers::MOVING; // Non moving only collides with moving
		case Layers::MOVING:
			return true; // Moving collides with everything
        case Layers::SENSORS:
            return inObject2 ==
                   Layers::MOVING; // Non moving only collides with moving
        default:
			JPH_ASSERT(false);
			return false;
		}
	}
};

// BroadPhaseLayerInterface implementation
// This defines a mapping between object and broadphase layers.
class BPLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface {
public:
	BPLayerInterfaceImpl()
	{
		// Create a mapping table from object to broad phase layer
		mObjectToBroadPhase[Layers::NON_MOVING] =
			BroadPhaseLayers::NON_MOVING;
		mObjectToBroadPhase[Layers::MOVING] = BroadPhaseLayers::MOVING;
        mObjectToBroadPhase[Layers::SENSORS] = BroadPhaseLayers::MOVING;
    }

	virtual uint GetNumBroadPhaseLayers() const override
	{
		return BroadPhaseLayers::NUM_LAYERS;
	}

	virtual JPH::BroadPhaseLayer
	GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override
	{
		JPH_ASSERT(inLayer < Layers::NUM_LAYERS);
		return mObjectToBroadPhase[inLayer];
	}

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
	virtual const char *
	GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const override
	{
		switch ((JPH::BroadPhaseLayer::Type)inLayer) {
		case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::NON_MOVING:
			return "NON_MOVING";
		case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::MOVING:
			return "MOVING";
		default:
			JPH_ASSERT(false);
			return "INVALID";
		}
	}
#endif // JPH_EXTERNAL_PROFILE || JPH_PROFILE_ENABLED

private:
	JPH::BroadPhaseLayer mObjectToBroadPhase[Layers::NUM_LAYERS];
};

/// Class that determines if an object layer can collide with a broadphase layer
class ObjectVsBroadPhaseLayerFilterImpl
	: public JPH::ObjectVsBroadPhaseLayerFilter {
public:
	virtual bool ShouldCollide(JPH::ObjectLayer inLayer1,
				   JPH::BroadPhaseLayer inLayer2) const override
	{
		switch (inLayer1) {
		case Layers::NON_MOVING:
			return inLayer2 == BroadPhaseLayers::MOVING;
		case Layers::MOVING:
			return true;
		default:
			JPH_ASSERT(false);
			return false;
		}
	}
};

class MyCollector : public JPH::CollideShapeBodyCollector {
public:
	MyCollector(JPH::PhysicsSystem *inSystem,
		    JPH::RVec3Arg inSurfacePosition,
		    JPH::Vec3Arg inSurfaceNormal, float inDeltaTime)
		: mSystem(inSystem)
		, mSurfacePosition(inSurfacePosition)
		, mSurfaceNormal(inSurfaceNormal)
		, mDeltaTime(inDeltaTime)
	{
	}

	virtual void AddHit(const JPH::BodyID &inBodyID) override
	{
		mInWater.insert(inBodyID);
	}

private:
	JPH::PhysicsSystem *mSystem;
	JPH::RVec3 mSurfacePosition;
	float mDeltaTime;

public:
	std::set<JPH::BodyID> mInWater;
	JPH::Vec3 mSurfaceNormal;
};

class DebugRenderer final : public JPH::DebugRendererSimple {
	Ogre::ManualObject *mObject;
	Ogre::SceneManager *mScnMgr;
	Ogre::SceneNode *mCameraNode;
	Ogre::MaterialPtr mat;
	struct line {
		Ogre::Vector3 from;
		Ogre::Vector3 to;
		Ogre::ColourValue c;
	};
	std::vector<struct line> mLines;
	struct tri {
		Ogre::Vector3 p[3];
		Ogre::ColourValue c;
	};
	std::vector<struct tri> mTriangles;

public:
	DebugRenderer(Ogre::SceneManager *scnMgr, Ogre::SceneNode *cameraNode);
	~DebugRenderer() override;
	void DrawLine(JPH::RVec3Arg inFrom, JPH::RVec3Arg inTo,
		      JPH::ColorArg inColor) override
	{
        JPH::Vec4 color = inColor.ToVec4();
        mLines.push_back(
            { { (float)inFrom[0], (float)inFrom[1],
                (float)inFrom[2] },
              { (float)inTo[0], (float)inTo[1], (float)inTo[2] },
              Ogre::ColourValue(color[0], color[1], color[2],
                        color[3]) });
	}
	void DrawTriangle(JPH::RVec3Arg inV1, JPH::RVec3Arg inV2,
			  JPH::RVec3Arg inV3, JPH::ColorArg inColor,
			  ECastShadow inCastShadow = ECastShadow::Off) override
	{
		Ogre::Vector3 d = mCameraNode->_getDerivedOrientation() *
				  Ogre::Vector3(0, 0, -1);
		JPH::Vec4 color = inColor.ToVec4();
		Ogre::Vector3 p1 = JoltPhysics::convert(inV1);
		Ogre::Vector3 p2 = JoltPhysics::convert(inV2);
		Ogre::Vector3 p3 = JoltPhysics::convert(inV3);
		Ogre::ColourValue cv(color[0], color[1], color[2], color[3]);

		mLines.push_back({ p1, p2, cv });
	}
	void DrawText3D(JPH::RVec3Arg inPosition,
			const std::string_view &inString,
			JPH::ColorArg inColor = JPH::Color::sWhite,
			float inHeight = 0.5f) override
	{
		std::cout << "text\n";
	}
	void finish()
	{
		Ogre::Vector3 d = mCameraNode->_getDerivedOrientation() *
				  Ogre::Vector3(0, 0, -1);
		int i;
		mObject->clear();
		mObject->begin(mat, Ogre::RenderOperation::OT_LINE_LIST);
		for (i = 0; i < mLines.size(); i++) {
			mObject->position(mLines[i].from);
			mObject->colour(mLines[i].c);
			mObject->position(mLines[i].to);
			mObject->colour(mLines[i].c);
		}
		mObject->end();
		mLines.clear();
		mTriangles.clear();
	}
};
DebugRenderer::DebugRenderer(Ogre::SceneManager *scnMgr,
			     Ogre::SceneNode *cameraNode)
	: mObject(scnMgr->createManualObject("joltDebugRenderer"))
	, mScnMgr(scnMgr)
	, mCameraNode(cameraNode)
	, mat(Ogre::MaterialManager::getSingleton().create("joltDebugDraw",
							   "General"))
{
	Ogre::Technique *technique = mat->getTechnique(0);
	Ogre::Pass *pass = technique->getPass(0);
	Ogre::ColourValue color(1, 0, 0, 1);
	pass->setCullingMode(Ogre::CullingMode::CULL_NONE);
	pass->setVertexColourTracking(Ogre::TVC_AMBIENT);
	pass->setLightingEnabled(false);
    pass->setDepthWriteEnabled(false);
    pass->setDepthCheckEnabled(false);
	DebugRenderer::Initialize();
	scnMgr->getRootSceneNode()->attachObject(mObject);
	mLines.reserve(6000);
	mObject->estimateVertexCount(64000);
	mObject->estimateIndexCount(8000);
    mObject->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
}
DebugRenderer::~DebugRenderer()
{
	mObject->getParentSceneNode()->detachObject(mObject);
	mScnMgr->destroyManualObject(mObject);
}

namespace JoltPhysics
{
struct ShapeData {
	JPH::ShapeRefC shape;
};
Ogre::Vector3 convert(const JPH::Vec3Arg &vec)
{
	return { vec[0], vec[1], vec[2] };
}
JPH::RVec3 convert(const Ogre::Vector3 &vec)
{
	return { vec.x, vec.y, vec.z };
}
Ogre::Quaternion convert(const JPH::QuatArg &rot)
{
	return { rot.GetW(), rot.GetX(), rot.GetY(), rot.GetZ() };
}
JPH::Quat convert(const Ogre::Quaternion &rot)
{
	return { rot.x, rot.y, rot.z, rot.w };
}
void CompoundShapeBuilder::addShape(JPH::ShapeRefC shape,
				    const Ogre::Vector3 &position,
				    const Ogre::Quaternion &rotation)
{
    shapeSettings.AddShape(JoltPhysics::convert<JPH::Vec3>(position),
			       JoltPhysics::convert(rotation), shape.GetPtr());
}
JPH::ShapeRefC CompoundShapeBuilder::build()
{
	JPH::ShapeSettings::ShapeResult result = shapeSettings.Create();
	return result.Get();
}
ContactListener::ContactListener()
	: JPH::ContactListener()
	, dispatch(nullptr)
{
}
JPH::ValidateResult ContactListener::OnContactValidate(
	const JPH::Body &inBody1, const JPH::Body &inBody2,
	JPH::RVec3Arg inBaseOffset,
	const JPH::CollideShapeResult &inCollisionResult)
{
	return JPH::ValidateResult::AcceptAllContactsForThisBodyPair;
}
void ContactListener::OnContactAdded(const JPH::Body &inBody1,
				     const JPH::Body &inBody2,
				     const JPH::ContactManifold &inManifold,
				     JPH::ContactSettings &ioSettings)
{
	reports.push_back({ true, inBody1.GetID(), inBody2.GetID(), inManifold,
			    ioSettings });
}
void ContactListener::OnContactPersisted(const JPH::Body &inBody1,
					 const JPH::Body &inBody2,
					 const JPH::ContactManifold &inManifold,
					 JPH::ContactSettings &ioSettings)
{
}
void ContactListener::OnContactRemoved(const JPH::SubShapeIDPair &inSubShapePair)
{
	reports.push_back({ false, inSubShapePair.GetBody1ID(),
			    inSubShapePair.GetBody2ID(), JPH::ContactManifold(),
			    JPH::ContactSettings() });
}
void ContactListener::update()
{
	for (auto contact : reports) {
		bool handled = false;
		if (listeners.find(contact.id1) != listeners.end()) {
			listeners[contact.id1](contact);
			handled = true;
		}
		if (listeners.find(contact.id2) != listeners.end()) {
			listeners[contact.id2](contact);
			handled = true;
		}
		if (!handled && dispatch) {
			dispatch(contact);
		}
	}
	reports.clear();
}
}

class Physics {
	JPH::PhysicsSystem physics_system;
	// We need a temp allocator for temporary allocations during the physics update. We're
	// pre-allocating 10 MB to avoid having to do allocations during the physics update.
	// B.t.w. 10 MB is way too much for this example but it is a typical value you can use.
	// If you don't want to pre-allocate you can also use TempAllocatorMalloc to fall back to
	// malloc / free.
	JPH::TempAllocatorImpl temp_allocator;
	// We need a job system that will execute physics jobs on multiple threads. Typically
	// you would implement the JobSystem interface yourself and let Jolt Physics run on top
	// of your own job scheduler. JobSystemThreadPool is an example implementation.
	JPH::JobSystemThreadPool job_system;

	// Create mapping table from object layer to broadphase layer
	// Note: As this is an interface, PhysicsSystem will take a reference to this so this instance needs to stay alive!
	// Also have a look at BroadPhaseLayerInterfaceTable or BroadPhaseLayerInterfaceMask for a simpler interface.
	BPLayerInterfaceImpl broad_phase_layer_interface;

	// Create class that filters object vs broadphase layers
	// Note: As this is an interface, PhysicsSystem will take a reference to this so this instance needs to stay alive!
	// Also have a look at ObjectVsBroadPhaseLayerFilterTable or ObjectVsBroadPhaseLayerFilterMask for a simpler interface.
	ObjectVsBroadPhaseLayerFilterImpl object_vs_broadphase_layer_filter;

	// Create class that filters object vs object layers
	// Note: As this is an interface, PhysicsSystem will take a reference to this so this instance needs to stay alive!
	// Also have a look at ObjectLayerPairFilterTable or ObjectLayerPairFilterMask for a simpler interface.
	ObjectLayerPairFilterImpl object_vs_object_layer_filter;

	DebugRenderer *mDebugRenderer;
	std::map<JPH::BodyID, Ogre::SceneNode *> id2node;
	std::map<Ogre::SceneNode *, JPH::BodyID> node2id;
	std::set<JPH::Character *> characters;
	std::set<JPH::BodyID> characterBodies;
	bool debugDraw;

public:
	class ActivationListener : public JPH::BodyActivationListener {
	public:
		virtual void OnBodyActivated(const JPH::BodyID &inBodyID,
					     JPH::uint64 inBodyUserData) = 0;
		virtual void OnBodyDeactivated(const JPH::BodyID &inBodyID,
					       JPH::uint64 inBodyUserData) = 0;
	};
	Physics(Ogre::SceneManager *scnMgr, Ogre::SceneNode *cameraNode,
		ActivationListener *activationListener = nullptr,
		JPH::ContactListener *contactListener = nullptr)
		: temp_allocator(10 * 1024 * 1024)
		, job_system(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers,
			     std::thread::hardware_concurrency() - 1)
		, mDebugRenderer(new DebugRenderer(scnMgr, cameraNode))
		, debugDraw(false)
	{
		// Create a factory, this class is responsible for creating instances of classes based on their name or hash and is mainly used for deserialization of saved data.
		// It is not directly used in this example but still required.
		JPH::Factory::sInstance = new JPH::Factory();

		// Register all physics types with the factory and install their collision handlers with the CollisionDispatch class.
		// If you have your own custom shape types you probably need to register their handlers with the CollisionDispatch before calling this function.
		// If you implement your own default material (PhysicsMaterial::sDefault) make sure to initialize it before this function or else this function will create one for you.
		JPH::RegisterTypes();

		// This is the max amount of rigid bodies that you can add to the physics system. If you try to add more you'll get an error.
		// Note: This value is low because this is a simple test. For a real project use something in the order of 65536.
		const uint cMaxBodies = 1024;

		// This determines how many mutexes to allocate to protect rigid bodies from concurrent access. Set it to 0 for the default settings.
		const uint cNumBodyMutexes = 0;

		// This is the max amount of body pairs that can be queued at any time (the broad phase will detect overlapping
		// body pairs based on their bounding boxes and will insert them into a queue for the narrowphase). If you make this buffer
		// too small the queue will fill up and the broad phase jobs will start to do narrow phase work. This is slightly less efficient.
		// Note: This value is low because this is a simple test. For a real project use something in the order of 65536.
		const uint cMaxBodyPairs = 1024;

		// This is the maximum size of the contact constraint buffer. If more contacts (collisions between bodies) are detected than this
		// number then these contacts will be ignored and bodies will start interpenetrating / fall through the world.
		// Note: This value is low because this is a simple test. For a real project use something in the order of 10240.
		const uint cMaxContactConstraints = 1024;

		// Now we can create the actual physics system.
		physics_system.Init(cMaxBodies, cNumBodyMutexes, cMaxBodyPairs,
				    cMaxContactConstraints,
				    broad_phase_layer_interface,
				    object_vs_broadphase_layer_filter,
				    object_vs_object_layer_filter);

		// A body activation listener gets notified when bodies activate and go to sleep
		// Note that this is called from a job so whatever you do here needs to be thread safe.
		// Registering one is entirely optional.
		if (activationListener)
			physics_system.SetBodyActivationListener(
				activationListener);

		// A contact listener gets notified when bodies (are about to) collide, and when they separate again.
		// Note that this is called from a job so whatever you do here needs to be thread safe.
		// Registering one is entirely optional.
		if (contactListener)
			physics_system.SetContactListener(contactListener);

		// The main way to interact with the bodies in the physics system is through the body interface. There is a locking and a non-locking
		// variant of this. We're going to use the locking version (even though we're not planning to access bodies from multiple threads)
		JPH::BodyInterface &body_interface =
			physics_system.GetBodyInterface();
		physics_system.SetGravity(JPH::Vec3(0, -0.1f, 0));
	}
	~Physics()
	{
		// Unregisters all types with the factory and cleans up the default material
		JPH::UnregisterTypes();

		// Destroy the factory
		delete JPH::Factory::sInstance;
		JPH::Factory::sInstance = nullptr;
	}
	float timeAccumulator = 0.0f;
	float fixedDeltaTime = 1.0f / 60.0f;
	void update(float dt)
	{
		JPH::BodyIDVector bodies;
		physics_system.GetBodies(bodies);
		JPH::BodyInterface &body_interface =
			physics_system.GetBodyInterface();
		for (JPH::BodyID bID : bodies) {
			if (id2node.find(bID) == id2node.end())
				continue;
			if (body_interface.GetMotionType(bID) ==
			    JPH::EMotionType::Kinematic) {
				Ogre::SceneNode *node = id2node[bID];
				body_interface.SetPositionAndRotationWhenChanged(
					bID,
					JoltPhysics::convert(
						node->_getDerivedPosition()),
					JoltPhysics::convert(
						node->_getDerivedOrientation()),
					JPH::EActivation::Activate);
			}
		}
		for (JPH::Character *ch : characters) {
			JPH::BodyID bID = ch->GetBodyID();
			if (id2node.find(bID) == id2node.end())
				continue;
			Ogre::SceneNode *node = id2node[bID];
			ch->SetRotation(JoltPhysics::convert(
				node->_getDerivedOrientation()));
		}
		int cCollisionSteps = 1;
		timeAccumulator += dt;
		if (debugDraw)
			cCollisionSteps = 4;
		while (timeAccumulator >= fixedDeltaTime) {
			physics_system.Update(dt, cCollisionSteps,
					      &temp_allocator, &job_system);
			timeAccumulator -= fixedDeltaTime;
		}
		for (JPH::BodyID bID : bodies) {
            JPH::RVec3 p;
			JPH::Quat q;
			if (id2node.find(bID) == id2node.end())
				continue;
			if (!body_interface.IsAdded(bID))
				continue;
			if (!body_interface.IsActive(bID))
				continue;
			if (body_interface.GetMotionType(bID) !=
			    JPH::EMotionType::Dynamic)
				continue;
			body_interface.GetPositionAndRotation(bID, p, q);
			Ogre::SceneNode *node = id2node[bID];
			node->_setDerivedPosition(JoltPhysics::convert(p));
			node->_setDerivedOrientation(JoltPhysics::convert(q));
		}
		for (JPH::Character *ch : characters) {
			if (body_interface.IsAdded(ch->GetBodyID()))
				ch->PostSimulation(0.1f);
		}

		if (debugDraw)
			physics_system.DrawBodies(
				JPH::BodyManager::DrawSettings(),
				mDebugRenderer);
		mDebugRenderer->finish();
		mDebugRenderer->NextFrame();
	}
	void setDebugDraw(bool enable)
	{
		debugDraw = enable;
	}
	static JPH::ShapeRefC createBoxShape(float x, float y, float z)
	{
        return new JPH::BoxShape(JPH::Vec3(x, y, z));
	}
	static JPH::ShapeRefC createCylinderShape(float halfHeight,
						  float radius)
	{
		return new JPH::CylinderShape(halfHeight, radius);
	}
	JPH::BodyCreationSettings createBodyCreationSettings(
        JPH::Shape *shape, JPH::RVec3 &position, JPH::Quat &rotation,
		JPH::EMotionType motionType, JPH::ObjectLayer layer)
	{
		JPH::BodyCreationSettings body_settings(
			shape, position, rotation, motionType, layer);
		return body_settings;
	}
	JPH::BodyCreationSettings
	createBodyCreationSettings(JPH::ShapeSettings *shapeSettings,
                   JPH::RVec3 &position, JPH::Quat &rotation,
				   JPH::EMotionType motionType,
				   JPH::ObjectLayer layer)
	{
		JPH::BodyCreationSettings body_settings(
			shapeSettings, position, rotation, motionType, layer);
		return body_settings;
	}
	void addBody(const JPH::BodyID &body, JPH::EActivation activation)
	{
		JPH::BodyInterface &body_interface =
			physics_system.GetBodyInterface();
		body_interface.AddBody(body, activation);
	}
	bool isAdded(const JPH::BodyID &body)
	{
		JPH::BodyInterface &body_interface =
			physics_system.GetBodyInterface();
		return body_interface.IsAdded(body);
	}
	void addAngularImpulse(const JPH::BodyID &id,
			       const Ogre::Vector3 &impulse)
	{
		JPH::BodyInterface &body_interface =
			physics_system.GetBodyInterface();
        body_interface.AddAngularImpulse(
            id, JoltPhysics::convert<JPH::Vec3>(impulse));
	}
	JPH::BodyID createBody(const JPH::BodyCreationSettings &settings,
			       ActivationListener *listener = nullptr)
	{
		JPH::BodyInterface &body_interface =
			physics_system.GetBodyInterface();
		JPH::Body *body = body_interface.CreateBody(settings);
		if (!body)
			return JPH::BodyID();
		return body->GetID();
	}
	void removeBody(const JPH::BodyID &id)
	{
		JPH::BodyInterface &body_interface =
			physics_system.GetBodyInterface();
		body_interface.RemoveBody(id);
	}
	void destroyBody(const JPH::BodyID &id)
	{
		JPH::BodyInterface &body_interface =
			physics_system.GetBodyInterface();
		body_interface.DestroyBody(id);
		node2id.erase(id2node[id]);
		id2node.erase(id);
	}
	JPH::ShapeRefC createShape(const JPH::ShapeSettings &settings)
	{
		JPH::ShapeSettings::ShapeResult result = settings.Create();
		JPH::ShapeRefC shape = result.Get();
		return shape;
	}
	JPH::BodyID createBody(const JPH::Shape *shape, float mass,
			       const Ogre::Vector3 &position,
			       const Ogre::Quaternion &rotation,
			       JPH::EMotionType motion, JPH::ObjectLayer layer,
			       ActivationListener *listener = nullptr)
	{
		JPH::BodyCreationSettings bodySettings(
			shape, JoltPhysics::convert(position),
			JoltPhysics::convert(rotation), motion, layer);
		if (mass > 0.001f) {
			JPH::MassProperties msp;
			msp.ScaleToMass(mass);
			bodySettings.mMassPropertiesOverride = msp;
		}
        JPH::BodyID id = createBody(bodySettings, listener);
        if (shape->GetType() == JPH::EShapeType::HeightField) {
            JPH::BodyInterface &body_interface =
                physics_system.GetBodyInterface();
            body_interface.SetFriction(id, 1.0f);
        }
        return id;
    }
	JPH::BodyID createBody(const JPH::Shape *shape, float mass,
			       Ogre::SceneNode *node, JPH::EMotionType motion,
			       JPH::ObjectLayer layer,
			       ActivationListener *listener = nullptr)
	{
		const Ogre::Vector3 &position = node->_getDerivedPosition();
		const Ogre::Quaternion &rotation =
			node->_getDerivedOrientation();
		std::cout << "body position: " << position << std::endl;
        JPH::BodyID id = createBody(shape, mass, position, rotation,
                        motion, layer, listener);
		id2node[id] = node;
		node2id[node] = id;
		return id;
	}
	JPH::BodyID createSensor(const JPH::Shape *shape,
				 const Ogre::Vector3 &position,
				 const Ogre::Quaternion &rotation,
				 JPH::EMotionType motion,
				 JPH::ObjectLayer layer)
	{
		JPH::BodyCreationSettings bodySettings(
			shape, JoltPhysics::convert(position),
			JoltPhysics::convert(rotation), motion, layer);
		bodySettings.mIsSensor = true;
		return createBody(bodySettings, nullptr);
	}
	JPH::BodyID createSensor(const JPH::Shape *shape, Ogre::SceneNode *node,
				 JPH::EMotionType motion,
				 JPH::ObjectLayer layer)
	{
		const Ogre::Vector3 &position = node->_getDerivedPosition();
		const Ogre::Quaternion &rotation =
			node->_getDerivedOrientation();
		std::cout << "body position: " << position << std::endl;
		JPH::BodyCreationSettings bodySettings(
			shape, JoltPhysics::convert(position),
			JoltPhysics::convert(rotation), motion, layer);
		bodySettings.mIsSensor = true;
		JPH::BodyID id = createBody(bodySettings);
		if (shape->GetType() == JPH::EShapeType::HeightField) {
			JPH::BodyInterface &body_interface =
				physics_system.GetBodyInterface();
			body_interface.SetFriction(id, 0.7f);
		}
		id2node[id] = node;
		node2id[node] = id;
		return id;
	}
	void addShapeToCompound(JPH::Ref<JPH::Shape> compoundShape,
				JPH::ShapeRefC childShape,
				const Ogre::Vector3 &position,
				const Ogre::Quaternion &rotation)
	{
		JPH::MutableCompoundShape *master =
			static_cast<JPH::MutableCompoundShape *>(
				compoundShape.GetPtr());
        master->AddShape(JoltPhysics::convert<JPH::Vec3>(position),
				 JoltPhysics::convert(rotation),
				 childShape.GetPtr());
	}
	class CharacterListener : public JPH::ContactListener {};
	JPH::CharacterBase *createCharacter(Ogre::SceneNode *node,
					    float characterHeight,
					    float characterRadius)
	{
		JPH::CharacterSettings settings;
		settings.mLayer = Layers::MOVING;
		float characterRadiusStanding = 0.2f;
		settings.mSupportingVolume =
			JPH::Plane(JPH::Vec3::sAxisY(), -0.2f);
		settings.mShape =
			JPH::RotatedTranslatedShapeSettings(
				JPH::Vec3(0,
					  0.5f * characterHeight +
						  characterRadius,
					  0),
				JPH::Quat::sIdentity(),
				new JPH::CapsuleShape(0.5f * characterHeight,
						      characterRadius))
				.Create()
				.Get();
		settings.mSupportingVolume =
			JPH::Plane(JPH::Vec3::sAxisY(), -characterRadius);
		JPH::Character *ch = new JPH::Character(
			&settings,
			JoltPhysics::convert(node->_getDerivedPosition()),
			JoltPhysics::convert(node->_getDerivedOrientation()), 0,
			&physics_system);
		JPH::BodyID id = ch->GetBodyID();
		id2node[id] = node;
		node2id[node] = id;
		characterBodies.insert(id);
		characters.insert(ch);
		return ch;
	}
	JPH::ShapeRefC createBoxShape(Ogre::Vector3 extents)
	{
		JPH::Vec3 h(extents.x, extents.y, extents.z);
		return new JPH::BoxShape(h);
	}
	JPH::ShapeRefC createSphereShape(float radius)
	{
		return new JPH::SphereShape(radius);
	}
	JPH::ShapeRefC createCapsuleShape(float halfHeightOfCylinder,
					  float radius)
	{
		return new JPH::CapsuleShape(halfHeightOfCylinder, radius);
	}
	JPH::ShapeRefC createMeshShape(Ogre::MeshPtr mesh)
	{
		JPH::VertexList vertices;
		JPH::IndexedTriangleList triangles;
		std::vector<uint32_t> indices;
		int count = mesh->getNumSubMeshes();
		int i, j;
		int indexCount = 0;
		int vertexCount = 0;
		int sharedVertexOffset = 0;
		for (i = 0; i < count; i++) {
			Ogre::SubMesh *submesh = mesh->getSubMesh(i);
			indexCount += submesh->indexData->indexCount;
			if (submesh->useSharedVertices)
				vertexCount +=
					mesh->sharedVertexData->vertexCount;
			else
				vertexCount += submesh->vertexData->vertexCount;
		}
		indices.reserve(indexCount);
		vertices.reserve(vertexCount);
		size_t currentVertexOffset = 0;
		bool added_shared = false;
		for (i = 0; i < count; i++) {
			Ogre::SubMesh *submesh = mesh->getSubMesh(i);
			Ogre::VertexData *vertex_data =
				submesh->useSharedVertices ?
					mesh->sharedVertexData :
					submesh->vertexData;
			bool add_vertices =
				(submesh->useSharedVertices && !added_shared) ||
				!submesh->useSharedVertices;
			if (add_vertices) {
				if (submesh->useSharedVertices)
					sharedVertexOffset = vertices.size();
				const Ogre::VertexDeclaration *decl =
					vertex_data->vertexDeclaration;
				const Ogre::VertexBufferBinding *bind =
					vertex_data->vertexBufferBinding;
				const Ogre::VertexElement *position_element =
					decl->findElementBySemantic(
						Ogre::VES_POSITION);
				if (!position_element)
					continue;
				Ogre::HardwareVertexBufferSharedPtr vbuf =
					bind->getBuffer(
						position_element->getSource());
				unsigned char *vertex_buffer = static_cast<
					unsigned char *>(vbuf->lock(
					Ogre::HardwareBuffer::HBL_READ_ONLY));
				int vertexSize = vbuf->getVertexSize();
				for (j = 0; j < vertex_data->vertexCount; j++) {
					float *position_data;
					position_element
						->baseVertexPointerToElement(
							vertex_buffer,
							&position_data);
					vertices.push_back(
						{ position_data[0],
						  position_data[1],
						  position_data[2] });
					vertex_buffer += vertexSize;
				}
				if (submesh->useSharedVertices)
					added_shared = true;
				vbuf->unlock();
			}
			Ogre::HardwareIndexBufferSharedPtr ibuf =
				submesh->indexData->indexBuffer;
			size_t numIndices = submesh->indexData->indexCount;
			size_t vertexOffset = submesh->useSharedVertices ?
						      sharedVertexOffset :
						      currentVertexOffset;
			if (ibuf->getType() ==
			    Ogre::HardwareIndexBuffer::IT_32BIT) {
				unsigned int *pIndices = static_cast<
					unsigned int *>(ibuf->lock(
					Ogre::HardwareBuffer::HBL_READ_ONLY));
				for (j = 0; j < numIndices; j++) {
					indices.push_back(
						(uint32_t)pIndices[j] +
						vertexOffset);
				}
				ibuf->unlock();
			} else {
				unsigned short *pIndices = static_cast<
					unsigned short *>(ibuf->lock(
					Ogre::HardwareBuffer::HBL_READ_ONLY));
				for (j = 0; j < numIndices; j++) {
					indices.push_back(
						(uint32_t)pIndices[j] +
						vertexOffset);
				}
				ibuf->unlock();
			}
			currentVertexOffset = vertices.size();
		}
		triangles.resize(indices.size() / 3);
		for (j = 0; j < indices.size() / 3; j++)
			triangles[j] = { indices[j * 3 + 0], indices[j * 3 + 1],
					 indices[j * 3 + 2] };
		JPH::MeshShapeSettings mesh_shape_settings(vertices, triangles);
		JPH::ShapeSettings::ShapeResult result =
			mesh_shape_settings.Create();
		OgreAssert(result.Get(), "Can not create mesh shape");
		return result.Get();
	}
	JPH::ShapeRefC createMeshShape(Ogre::String meshName)
	{
		Ogre::DefaultHardwareBufferManager dmgr;
		Ogre::MeshPtr mesh =
			Ogre::MeshManager::getSingleton().getByName(meshName);
		if (!mesh->isLoaded()) {
			mesh->setHardwareBufferManager(&dmgr);
			mesh->load();
		}
		return createMeshShape(mesh);
	}
	JPH::ShapeRefC createConvexHullShape(Ogre::MeshPtr mesh)
	{
		std::vector<JPH::Vec3> vertices;
		JPH::IndexedTriangleList triangles;
		std::vector<uint32_t> indices;
		int count = mesh->getNumSubMeshes();
		int i, j;
		int indexCount = 0;
		int vertexCount = 0;
		int sharedVertexOffset = 0;
		for (i = 0; i < count; i++) {
			Ogre::SubMesh *submesh = mesh->getSubMesh(i);
			indexCount += submesh->indexData->indexCount;
			if (submesh->useSharedVertices)
				vertexCount +=
					mesh->sharedVertexData->vertexCount;
			else
				vertexCount += submesh->vertexData->vertexCount;
		}
		indices.reserve(indexCount);
		vertices.reserve(vertexCount);
		size_t currentVertexOffset = 0;
		bool added_shared = false;
		for (i = 0; i < count; i++) {
			Ogre::SubMesh *submesh = mesh->getSubMesh(i);
			Ogre::VertexData *vertex_data =
				submesh->useSharedVertices ?
					mesh->sharedVertexData :
					submesh->vertexData;
			bool add_vertices =
				(submesh->useSharedVertices && !added_shared) ||
				!submesh->useSharedVertices;
			if (add_vertices) {
				if (submesh->useSharedVertices)
					sharedVertexOffset = vertices.size();
				const Ogre::VertexDeclaration *decl =
					vertex_data->vertexDeclaration;
				const Ogre::VertexBufferBinding *bind =
					vertex_data->vertexBufferBinding;
				const Ogre::VertexElement *position_element =
					decl->findElementBySemantic(
						Ogre::VES_POSITION);
				if (!position_element)
					continue;
				Ogre::HardwareVertexBufferSharedPtr vbuf =
					bind->getBuffer(
						position_element->getSource());
				unsigned char *vertex_buffer = static_cast<
					unsigned char *>(vbuf->lock(
					Ogre::HardwareBuffer::HBL_READ_ONLY));
				int vertexSize = vbuf->getVertexSize();
				for (j = 0; j < vertex_data->vertexCount; j++) {
					float *position_data;
					position_element
						->baseVertexPointerToElement(
							vertex_buffer,
							&position_data);
					vertices.push_back(
						{ position_data[0],
						  position_data[1],
						  position_data[2] });
					vertex_buffer += vertexSize;
				}
				if (submesh->useSharedVertices)
					added_shared = true;
				vbuf->unlock();
			}
			Ogre::HardwareIndexBufferSharedPtr ibuf =
				submesh->indexData->indexBuffer;
			size_t numIndices = submesh->indexData->indexCount;
			size_t vertexOffset = submesh->useSharedVertices ?
						      sharedVertexOffset :
						      currentVertexOffset;
			if (ibuf->getType() ==
			    Ogre::HardwareIndexBuffer::IT_32BIT) {
				unsigned int *pIndices = static_cast<
					unsigned int *>(ibuf->lock(
					Ogre::HardwareBuffer::HBL_READ_ONLY));
				for (j = 0; j < numIndices; j++) {
					indices.push_back(
						(uint32_t)pIndices[j] +
						vertexOffset);
				}
				ibuf->unlock();
			} else {
				unsigned short *pIndices = static_cast<
					unsigned short *>(ibuf->lock(
					Ogre::HardwareBuffer::HBL_READ_ONLY));
				for (j = 0; j < numIndices; j++) {
					indices.push_back(
						(uint32_t)pIndices[j] +
						vertexOffset);
				}
				ibuf->unlock();
			}
			currentVertexOffset = vertices.size();
		}
		triangles.resize(indices.size() / 3);
		for (j = 0; j < indices.size() / 3; j++)
			triangles[j] = { indices[j * 3 + 0], indices[j * 3 + 1],
					 indices[j * 3 + 2] };
		JPH::ConvexHullShapeSettings mesh_shape_settings(
			vertices.data(), vertices.size());
		JPH::ShapeSettings::ShapeResult result =
			mesh_shape_settings.Create();
		OgreAssert(result.Get(), "Can not create mesh shape");
		return result.Get();
	}
	JPH::ShapeRefC createConvexHullShape(Ogre::String meshName)
	{
		auto p = Ogre::DefaultHardwareBufferManager::getSingletonPtr();
		if (!p) {
			new Ogre::DefaultHardwareBufferManager;
			p = Ogre::DefaultHardwareBufferManager::getSingletonPtr();
		}
		Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().load(
			meshName, "General");
		if (mesh.get()) {
			if (!mesh->isLoaded()) {
				mesh->setHardwareBufferManager(p);
				mesh->load();
			}
			return createConvexHullShape(mesh);
		}
		OgreAssert(mesh.get(), "No file " + meshName);
		return JPH::ShapeRefC();
	}
	/// Create a height field shape of inSampleCount * inSampleCount vertices.
	/// The height field is a surface defined by: inOffset + inScale * (x, inSamples[y * inSampleCount + x], y).
	/// where x and y are integers in the range x and y e [0, inSampleCount - 1].
	/// inSampleCount: inSampleCount / mBlockSize must be minimally 2 and a power of 2 is the most efficient in terms of performance and storage.
	/// inSamples: inSampleCount^2 vertices.
	/// inMaterialIndices: (inSampleCount - 1)^2 indices that index into inMaterialList.
	JPH::ShapeRefC createHeightfieldShape(const float *samples,
					      Ogre::Vector3 offset,
					      Ogre::Vector3 scale,
					      int sampleCount)
	{
		int i;
		JPH::HeightFieldShapeSettings heightfieldSettings(
            samples, JoltPhysics::convert<JPH::Vec3>(offset),
            JoltPhysics::convert<JPH::Vec3>(scale),
            (uint32_t)sampleCount);
		for (i = 0; i < sampleCount; i++) {
			memcpy(heightfieldSettings.mHeightSamples.data() +
				       sampleCount * i,
			       samples + sampleCount * (sampleCount - i - 1),
			       sizeof(float) * sampleCount);
		}
		JPH::ShapeSettings::ShapeResult result =
			heightfieldSettings.Create();
		OgreAssert(result.Get(), "Can not create heightfield shape");
		return result.Get();
	}
	JPH::ShapeRefC createMutableCompoundShape(
		const std::vector<JPH::ShapeRefC> &shapes,
		const std::vector<Ogre::Vector3> &positions,
		const std::vector<Ogre::Quaternion> &rotations)
	{
		int i;
		OgreAssert(shapes.size() == positions.size() &&
				   shapes.size() == rotations.size(),
			   "bad parameters");
		JPH::MutableCompoundShapeSettings settings;
		for (i = 0; i < shapes.size(); i++)
            settings.AddShape(
                JoltPhysics::convert<JPH::Vec3>(positions[i]),
                JoltPhysics::convert(rotations[i]),
                shapes[i].GetPtr());
		JPH::ShapeSettings::ShapeResult result = settings.Create();
		OgreAssert(result.Get(), "Can not create compound shape");
		return result.Get();
	}
	JPH::ShapeRefC createStaticCompoundShape(
		const std::vector<JPH::ShapeRefC> &shapes,
		const std::vector<Ogre::Vector3> &positions,
		const std::vector<Ogre::Quaternion> &rotations)
	{
		int i;
		OgreAssert(shapes.size() == positions.size() &&
				   shapes.size() == rotations.size(),
			   "bad parameters");
		JPH::StaticCompoundShapeSettings settings;
		for (i = 0; i < shapes.size(); i++)
            settings.AddShape(
                JoltPhysics::convert<JPH::Vec3>(positions[i]),
                JoltPhysics::convert(rotations[i]),
                shapes[i].GetPtr());
		JPH::ShapeSettings::ShapeResult result = settings.Create();
		OgreAssert(result.Get(), "Can not create compound shape");
		return result.Get();
	}
	JPH::ShapeRefC
	createOffsetCenterOfMassShape(const Ogre::Vector3 &offset,
				      JPH::ShapeRefC shape)
	{
		JPH::OffsetCenterOfMassShapeSettings settings(
            JoltPhysics::convert<JPH::Vec3>(offset),
            shape.GetPtr());
		JPH::ShapeSettings::ShapeResult result = settings.Create();
		OgreAssert(result.Get(), "Can not create com offset shape");
		return result.Get();
	}
    JPH::ShapeRefC
    createRotatedTranslatedShape(const Ogre::Vector3 &offset,
                     const Ogre::Quaternion rotation,
                     JPH::ShapeRefC shape)
    {
        return JPH::RotatedTranslatedShapeSettings(
                   JoltPhysics::convert<JPH::Vec3>(offset),
                   JoltPhysics::convert(rotation), shape)
            .Create()
            .Get();
    }
    void applyBuoyancyImpulse(JPH::BodyID id,
				  const Ogre::Vector3 &surfacePosition,
				  const Ogre::Vector3 &surfaceNormal,
				  float buoyancy, float linearDrag,
				  float angularDrag,
				  const Ogre::Vector3 &fluidVelocity,
				  const Ogre::Vector3 &gravity, float dt)
	{
		JPH::BodyLockWrite lock(physics_system.GetBodyLockInterface(),
					id);
		JPH::Body &body = lock.GetBody();
        body.ApplyBuoyancyImpulse(
            JoltPhysics::convert(surfacePosition),
            JoltPhysics::convert<JPH::Vec3>(surfaceNormal),
            buoyancy, linearDrag, angularDrag,
            JoltPhysics::convert<JPH::Vec3>(fluidVelocity),
            JoltPhysics::convert<JPH::Vec3>(gravity), dt);
	}
	void applyBuoyancyImpulse(JPH::BodyID id,
				  const Ogre::Vector3 &surfacePosition,
				  const Ogre::Vector3 &surfaceNormal,
				  float buoyancy, float linearDrag,
				  float angularDrag,
				  const Ogre::Vector3 &fluidVelocity, float dt)
	{
		JPH::BodyLockWrite lock(physics_system.GetBodyLockInterface(),
					id);
		JPH::Body &body = lock.GetBody();
        body.ApplyBuoyancyImpulse(
            JoltPhysics::convert(surfacePosition),
            JoltPhysics::convert<JPH::Vec3>(surfaceNormal),
            buoyancy, linearDrag, angularDrag,
            JoltPhysics::convert<JPH::Vec3>(fluidVelocity),
            physics_system.GetGravity(), dt);
	}
	bool isActive(JPH::BodyID id)
	{
		return physics_system.GetBodyInterface().IsActive(id);
	}
	void activate(JPH::BodyID id)
	{
		return physics_system.GetBodyInterface().ActivateBody(id);
	}
	Ogre::Vector3 getPosition(JPH::BodyID id)
	{
		return JoltPhysics::convert(
			physics_system.GetBodyInterface().GetPosition(id));
	}
	Ogre::Quaternion getRotation(JPH::BodyID id)
	{
		return JoltPhysics::convert(
			physics_system.GetBodyInterface().GetRotation(id));
	}
	void getPositionAndRotation(JPH::BodyID id, Ogre::Vector3 &position,
				    Ogre::Quaternion &rotation)
	{
        JPH::RVec3 _position;
		JPH::Quat _rotation;
		physics_system.GetBodyInterface().GetPositionAndRotation(
			id, _position, _rotation);
		position = JoltPhysics::convert(_position);
		rotation = JoltPhysics::convert(_rotation);
	}
	void setPosition(JPH::BodyID id, const Ogre::Vector3 &position,
			 bool activate = true)
	{
		physics_system.GetBodyInterface().SetPosition(
			id, JoltPhysics::convert(position),
			activate ? JPH::EActivation::Activate :
				   JPH::EActivation::DontActivate);
	}
	void setRotation(JPH::BodyID id, const Ogre::Quaternion &rotation,
			 bool activate = true)
	{
		physics_system.GetBodyInterface().SetRotation(
			id, JoltPhysics::convert(rotation),
			activate ? JPH::EActivation::Activate :
				   JPH::EActivation::DontActivate);
	}
	void setPositionAndRotation(JPH::BodyID id,
				    const Ogre::Vector3 &position,
				    const Ogre::Quaternion &rotation,
				    bool activate = true)
	{
		physics_system.GetBodyInterface().SetPositionAndRotation(
			id, JoltPhysics::convert(position),
			JoltPhysics::convert(rotation),
			activate ? JPH::EActivation::Activate :
				   JPH::EActivation::DontActivate);
	}
	Ogre::Vector3 getLinearVelocity(JPH::BodyID id)
	{
		return JoltPhysics::convert(
			physics_system.GetBodyInterface().GetLinearVelocity(
				id));
	}
	Ogre::Vector3 getAngularVelocity(JPH::BodyID id)
	{
		return JoltPhysics::convert(
			physics_system.GetBodyInterface().GetAngularVelocity(
				id));
	}
	float getFriction(JPH::BodyID id)
	{
		return physics_system.GetBodyInterface().GetFriction(id);
	}
	void setFriction(JPH::BodyID id, float friction)
	{
		return physics_system.GetBodyInterface().SetFriction(id,
								     friction);
	}
	void broadphaseQuery(float dt, const Ogre::Vector3 &position,
			     std::set<JPH::BodyID> &inWater)
	{
        JPH::RVec3 surface_point = JoltPhysics::convert(
			position + Ogre::Vector3(0, -0.1f, 0));

		MyCollector collector(&physics_system, surface_point,
                      JPH::Vec3::sAxisY(), dt);
		// Apply buoyancy to all bodies that intersect with the water
		JPH::AABox water_box(-JPH::Vec3(1000, 1000, 1000),
				     JPH::Vec3(1000, 0.1f, 1000));
		water_box.Translate(JPH::Vec3(surface_point));
		physics_system.GetBroadPhaseQuery().CollideAABox(
			water_box, collector,
			JPH::SpecifiedBroadPhaseLayerFilter(
				BroadPhaseLayers::MOVING),
			JPH::SpecifiedObjectLayerFilter(Layers::MOVING));
		inWater.clear();
		for (JPH::BodyID inBodyID : collector.mInWater) {
			inWater.insert(inBodyID);
		}
	}
	bool raycastQuery(Ogre::Vector3 startPoint, Ogre::Vector3 endPoint,
              Ogre::Vector3 &position, JPH::BodyID &id)
	{
		int i;
		Ogre::Vector3 direction = endPoint - startPoint;
		JPH::RRayCast ray{ JoltPhysics::convert(startPoint),
                   JoltPhysics::convert<JPH::Vec3>(direction) };
		JPH::RayCastResult hit;
		bool hadHit = physics_system.GetNarrowPhaseQuery().CastRay(
			ray, hit, {},
			JPH::SpecifiedObjectLayerFilter(Layers::NON_MOVING));
        if (hadHit) {
			position = JoltPhysics::convert(
				ray.GetPointOnRay(hit.mFraction));
            id = hit.mBodyID;
        }
		return hadHit;
	}
};

static Physics *phys = nullptr;
JoltPhysicsWrapper::JoltPhysicsWrapper(Ogre::SceneManager *scnMgr,
				       Ogre::SceneNode *cameraNode)
	: Ogre::Singleton<JoltPhysicsWrapper>()
{
	// Register allocation hook. In this example we'll just let Jolt use malloc / free but you can override these if you want (see Memory.h).
	// This needs to be done before any other Jolt function is called.
	JPH::RegisterDefaultAllocator();

	// Install trace and assert callbacks
	JPH::Trace = TraceImpl;
	JPH_IF_ENABLE_ASSERTS(JPH::AssertFailed = AssertFailedImpl;)

	phys = new Physics(scnMgr, cameraNode, nullptr, &contacts);
}

JoltPhysicsWrapper::~JoltPhysicsWrapper()
{
	if (phys)
		delete phys;
}

void JoltPhysicsWrapper::update(float dt)
{
	phys->update(dt);
	contacts.update();
}

void JoltPhysicsWrapper::addBody(const JPH::BodyID &body,
				 JPH::EActivation activation)
{
	phys->addBody(body, activation);
}

bool JoltPhysicsWrapper::isAdded(const JPH::BodyID &body)
{
	return phys->isAdded(body);
}

JPH::ShapeRefC JoltPhysicsWrapper::createBoxShape(const Ogre::Vector3 &extents)
{
	return phys->createBoxShape(extents);
}

JPH::ShapeRefC JoltPhysicsWrapper::createSphereShape(float radius)
{
	return phys->createSphereShape(radius);
}

JPH::ShapeRefC JoltPhysicsWrapper::createCylinderShape(float halfHeight,
						       float radius)
{
	return phys->createCylinderShape(halfHeight, radius);
}

JPH::ShapeRefC JoltPhysicsWrapper::createMeshShape(Ogre::MeshPtr mesh)
{
	return phys->createMeshShape(mesh);
}

JPH::ShapeRefC JoltPhysicsWrapper::createMeshShape(Ogre::String meshName)
{
	return phys->createMeshShape(meshName);
}

JPH::ShapeRefC JoltPhysicsWrapper::createConvexHullShape(Ogre::MeshPtr mesh)
{
	return phys->createConvexHullShape(mesh);
}

JPH::ShapeRefC JoltPhysicsWrapper::createConvexHullShape(Ogre::String meshName)
{
	return phys->createConvexHullShape(meshName);
}

JPH::ShapeRefC JoltPhysicsWrapper::createHeightfieldShape(const float *samples,
							  Ogre::Vector3 offset,
							  Ogre::Vector3 scale,
							  int sampleCount)
{
	return phys->createHeightfieldShape(samples, offset, scale,
					    sampleCount);
}

JPH::ShapeRefC JoltPhysicsWrapper::createMutableCompoundShape(
	const std::vector<JPH::ShapeRefC> &shapes,
	const std::vector<Ogre::Vector3> &positions,
	const std::vector<Ogre::Quaternion> &rotations)
{
	return phys->createMutableCompoundShape(shapes, positions, rotations);
}

JPH::ShapeRefC JoltPhysicsWrapper::createStaticCompoundShape(
	const std::vector<JPH::ShapeRefC> &shapes,
	const std::vector<Ogre::Vector3> &positions,
	const std::vector<Ogre::Quaternion> &rotations)
{
	return phys->createStaticCompoundShape(shapes, positions, rotations);
}

JPH::ShapeRefC
JoltPhysicsWrapper::createOffsetCenterOfMassShape(const Ogre::Vector3 &offset,
						  JPH::ShapeRefC shape)
{
    return phys->createOffsetCenterOfMassShape(offset, shape);
}

JPH::ShapeRefC JoltPhysicsWrapper::createRotatedTranslatedShape(
    const Ogre::Vector3 &offset, const Ogre::Quaternion rotation,
    JPH::ShapeRefC shape)
{
    return phys->createRotatedTranslatedShape(offset, rotation, shape);
}

JPH::BodyID
JoltPhysicsWrapper::createBody(const JPH::BodyCreationSettings &settings)
{
	return phys->createBody(settings);
}
JPH::BodyID JoltPhysicsWrapper::createBody(const JPH::Shape *shape, float mass,
					   const Ogre::Vector3 &position,
					   const Ogre::Quaternion &rotation,
					   JPH::EMotionType motion,
					   JPH::ObjectLayer layer)
{
	return phys->createBody(shape, mass, position, rotation, motion, layer);
}
JPH::BodyID JoltPhysicsWrapper::createBody(const JPH::Shape *shape, float mass,
					   Ogre::SceneNode *node,
					   JPH::EMotionType motion,
					   JPH::ObjectLayer layer)
{
	return phys->createBody(shape, mass, node, motion, layer);
}
JPH::BodyID JoltPhysicsWrapper::createSensor(const JPH::Shape *shape,
					     const Ogre::Vector3 &position,
					     const Ogre::Quaternion &rotation,
					     JPH::EMotionType motion,
					     JPH::ObjectLayer layer)
{
	return phys->createSensor(shape, position, rotation, motion, layer);
}
JPH::BodyID JoltPhysicsWrapper::createSensor(const JPH::Shape *shape,
					     Ogre::SceneNode *node,
					     JPH::EMotionType motion,
					     JPH::ObjectLayer layer)
{
	return phys->createSensor(shape, node, motion, layer);
}
JPH::CharacterBase *JoltPhysicsWrapper::createCharacter(Ogre::SceneNode *node,
							float characterHeight,
							float characterRadius)
{
	return phys->createCharacter(node, characterHeight, characterRadius);
}
void JoltPhysicsWrapper::addShapeToCompound(JPH::Ref<JPH::Shape> compoundShape,
					    JPH::ShapeRefC childShape,
					    const Ogre::Vector3 &position,
					    const Ogre::Quaternion &rotation)
{
	phys->addShapeToCompound(compoundShape, childShape, position, rotation);
}
void JoltPhysicsWrapper::removeBody(const JPH::BodyID &id)
{
	phys->removeBody(id);
}
void JoltPhysicsWrapper::destroyBody(const JPH::BodyID &id)
{
	phys->destroyBody(id);
}
void JoltPhysicsWrapper::setDebugDraw(bool enable)
{
	phys->setDebugDraw(enable);
}
void JoltPhysicsWrapper::broadphaseQuery(float dt,
					 const Ogre::Vector3 &position,
					 std::set<JPH::BodyID> &inWater)
{
	phys->broadphaseQuery(dt, position, inWater);
}
void JoltPhysicsWrapper::applyBuoyancyImpulse(
	JPH::BodyID id, const Ogre::Vector3 &surfacePosition,
	const Ogre::Vector3 &surfaceNormal, float buoyancy, float linearDrag,
	float angularDrag, const Ogre::Vector3 &fluidVelocity,
	const Ogre::Vector3 &gravity, float dt)
{
	phys->applyBuoyancyImpulse(id, surfacePosition, surfaceNormal, buoyancy,
				   linearDrag, angularDrag, fluidVelocity,
				   gravity, dt);
}
void JoltPhysicsWrapper::applyBuoyancyImpulse(
	JPH::BodyID id, const Ogre::Vector3 &surfacePosition,
	const Ogre::Vector3 &surfaceNormal, float buoyancy, float linearDrag,
	float angularDrag, const Ogre::Vector3 &fluidVelocity, float dt)
{
	phys->applyBuoyancyImpulse(id, surfacePosition, surfaceNormal, buoyancy,
				   linearDrag, angularDrag, fluidVelocity, dt);
}
bool JoltPhysicsWrapper::isActive(JPH::BodyID id)
{
	return phys->isActive(id);
}
void JoltPhysicsWrapper::activate(JPH::BodyID id)
{
	phys->activate(id);
}
Ogre::Vector3 JoltPhysicsWrapper::getPosition(JPH::BodyID id)
{
	return phys->getPosition(id);
}
void JoltPhysicsWrapper::setPosition(JPH::BodyID id,
				     const Ogre::Vector3 &position,
				     bool activate)
{
	return phys->setPosition(id, position, activate);
}
Ogre::Quaternion JoltPhysicsWrapper::getRotation(JPH::BodyID id)
{
	return phys->getRotation(id);
}
void JoltPhysicsWrapper::setRotation(JPH::BodyID id,
				     const Ogre::Quaternion &rotation,
				     bool activate)
{
	phys->setRotation(id, rotation, activate);
}
void JoltPhysicsWrapper::getPositionAndRotation(JPH::BodyID id,
						Ogre::Vector3 &position,
						Ogre::Quaternion &rotation)
{
	phys->getPositionAndRotation(id, position, rotation);
}
void JoltPhysicsWrapper::setPositionAndRotation(
	JPH::BodyID id, const Ogre::Vector3 &position,
	const Ogre::Quaternion &rotation, bool activate)
{
	phys->setPositionAndRotation(id, position, rotation, activate);
}
Ogre::Vector3 JoltPhysicsWrapper::getLinearVelocity(JPH::BodyID id)
{
	return phys->getLinearVelocity(id);
}
Ogre::Vector3 JoltPhysicsWrapper::getAngularVelocity(JPH::BodyID id)
{
	return phys->getAngularVelocity(id);
}
float JoltPhysicsWrapper::getFriction(JPH::BodyID id)
{
	return phys->getFriction(id);
}
void JoltPhysicsWrapper::setFriction(JPH::BodyID id, float friction)
{
	phys->setFriction(id, friction);
}
void JoltPhysicsWrapper::addAngularImpulse(const JPH::BodyID &id,
					   const Ogre::Vector3 &impulse)
{
	return phys->addAngularImpulse(id, impulse);
}
void JoltPhysicsWrapper::setDispatch(
	std::function<
		void(const JoltPhysics::ContactListener::ContactReport &report)>
		dispatcher)
{
	contacts.setDispatch(dispatcher);
}
void JoltPhysicsWrapper::addContactListener(
	const JPH::BodyID &id,
	std::function<
		void(const JoltPhysics::ContactListener::ContactReport &report)>
		listener)
{
	contacts.addListener(id, listener);
}
void JoltPhysicsWrapper::removeContactListener(const JPH::BodyID &id)
{
	contacts.removeListener(id);
}
bool JoltPhysicsWrapper::raycastQuery(Ogre::Vector3 startPoint,
				      Ogre::Vector3 endPoint,
                      Ogre::Vector3 &position, JPH::BodyID &id)
{
    return phys->raycastQuery(startPoint, endPoint, position, id);
}
template <>
JoltPhysicsWrapper *Ogre::Singleton<JoltPhysicsWrapper>::msSingleton = 0;
