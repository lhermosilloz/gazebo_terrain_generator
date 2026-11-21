/*
 * GroundFriction - Gazebo Harmonic (gz-sim 8) system plugin.
 *
 * Why this exists
 * ---------------
 * gz-sim's default physics engine (DART, via gz-physics) never reads
 * <surface><friction> for heightmap or mesh collisions: gz-sim attaches those
 * shapes with AttachHeightmapShape / AttachMeshShape, which create the DART
 * shape node with default dynamics (mu = mu2 = 1), and the SDF <surface> block
 * is only applied to primitive shapes (box, sphere, ...). DART then uses
 * min(mu_a, mu_b) for every contact, so a heightmap ground is always mu = 1.
 *
 * This system uses gz-sim's contact-surface customization hook (the same one
 * the TrackController system uses) to set the friction coefficients of every
 * contact that involves a collision of the model it is attached to.
 *
 * Usage (inside the terrain <model>):
 *
 *   <plugin filename="GroundFriction"
 *           name="gazebo_terrain_generator::GroundFriction">
 *     <mu>0.4</mu>     <!-- required, >= 0 -->
 *     <mu2>0.4</mu2>   <!-- optional, defaults to <mu> -->
 *   </plugin>
 *
 * Rules applied to each contact between collisions A and B:
 *  - A collision of this model (including nested models such as buildings)
 *    gets <mu>/<mu2> from the plugin, unless that collision declares its own
 *    <surface><friction><ode><mu> in SDF, which then wins.
 *  - Any other collision uses its own SDF <ode><mu>/<mu2> (default 1).
 *  - The contact uses min(A, B) for each direction, which is DART's own rule.
 * Contacts that involve no collision of this model are left untouched.
 */

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

#include <gz/common/Console.hh>
#include <gz/common/Event.hh>
#include <gz/math/Vector3.hh>
#include <gz/physics/ContactProperties.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/physics/Events.hh>
#include <sdf/Collision.hh>
#include <sdf/Element.hh>

namespace gazebo_terrain_generator
{
using gz::sim::Entity;
using gz::sim::EntityComponentManager;
using gz::sim::EventManager;
using gz::sim::UpdateInfo;
using gz::sim::kNullEntity;
namespace components = gz::sim::components;

class GroundFriction : public gz::sim::System,
                       public gz::sim::ISystemConfigure,
                       public gz::sim::ISystemPreUpdate
{
  private: struct Coeffs
  {
    double mu{1.0};
    double mu2{1.0};
  };

  // ---------------------------------------------------------------------
  public: void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &_eventMgr) override
  {
    if (!_ecm.Component<components::Model>(_entity))
    {
      gzerr << "[GroundFriction] must be attached to a <model>; "
            << "friction will not be changed." << std::endl;
      return;
    }
    if (!_sdf || !_sdf->HasElement("mu"))
    {
      gzerr << "[GroundFriction] missing required <mu>; "
            << "friction will not be changed." << std::endl;
      return;
    }
    const double mu = _sdf->Get<double>("mu");
    const double mu2 = _sdf->HasElement("mu2") ? _sdf->Get<double>("mu2") : mu;
    if (!std::isfinite(mu) || !std::isfinite(mu2) || mu < 0.0 || mu2 < 0.0)
    {
      gzerr << "[GroundFriction] <mu>/<mu2> must be finite and >= 0; "
            << "friction will not be changed." << std::endl;
      return;
    }

    this->model = _entity;
    this->ground = {mu, mu2};

    using P = gz::physics::FeaturePolicy3d;
    using F = gz::physics::SetContactPropertiesCallbackFeature;
    this->connection = _eventMgr.Connect<
        gz::sim::events::CollectContactSurfaceProperties>(
        [this](const Entity &_collision1, const Entity &_collision2,
               const gz::math::Vector3d & /*_point*/,
               const std::optional<gz::math::Vector3d> /*_force*/,
               const std::optional<gz::math::Vector3d> /*_normal*/,
               const std::optional<double> /*_depth*/,
               const size_t /*_numContactsOnCollision*/,
               F::ContactSurfaceParams<P> &_params)
        {
          this->OnContact(_collision1, _collision2, _params);
        });

    // Collisions that already exist (the model's own ones do by now).
    _ecm.Each<components::Collision, components::CollisionElement>(
        [&](const Entity &_coll, const components::Collision *,
            const components::CollisionElement *_elem) -> bool
        {
          this->Register(_ecm, _coll, _elem->Data());
          return true;
        });

    auto nameComp = _ecm.Component<components::Name>(_entity);
    gzmsg << "[GroundFriction] model [" << (nameComp ? nameComp->Data() : "")
          << "]: mu=" << mu << " mu2=" << mu2 << " applied to "
          << this->managed.size() << " collision(s)" << std::endl;
  }

  // ---------------------------------------------------------------------
  public: void PreUpdate(const UpdateInfo & /*_info*/,
                         EntityComponentManager &_ecm) override
  {
    if (this->model == kNullEntity)
      return;

    _ecm.EachNew<components::Collision, components::CollisionElement>(
        [&](const Entity &_coll, const components::Collision *,
            const components::CollisionElement *_elem) -> bool
        {
          this->Register(_ecm, _coll, _elem->Data());
          return true;
        });

    _ecm.EachRemoved<components::Collision>(
        [&](const Entity &_coll, const components::Collision *) -> bool
        {
          this->managed.erase(_coll);
          this->others.erase(_coll);
          return true;
        });
  }

  // ---------------------------------------------------------------------
  /// Friction explicitly written in a collision's SDF, if any.
  private: static std::optional<Coeffs> ExplicitFriction(
               const sdf::Collision &_collision)
  {
    sdf::ElementPtr elem = _collision.Element();
    if (!elem)
      return std::nullopt;
    // FindElement never creates elements, unlike GetElement.
    sdf::ElementPtr ode;
    if (auto surface = elem->FindElement("surface"))
      if (auto friction = surface->FindElement("friction"))
        ode = friction->FindElement("ode");
    if (!ode)
      return std::nullopt;
    auto muElem = ode->FindElement("mu");
    auto mu2Elem = ode->FindElement("mu2");
    if (!muElem && !mu2Elem)
      return std::nullopt;
    // Mirror the dartsim plugin: <mu> sets both directions, <mu2> overrides
    // the second one, anything unset stays at DART's default of 1.
    Coeffs c;
    if (muElem)
      c.mu = c.mu2 = muElem->Get<double>();
    if (mu2Elem)
      c.mu2 = mu2Elem->Get<double>();
    return c;
  }

  // ---------------------------------------------------------------------
  /// True if _entity is (nested) below this->model.
  private: bool InModel(const EntityComponentManager &_ecm,
                        Entity _entity) const
  {
    for (int depth = 0; depth < 64 && _entity != kNullEntity; ++depth)
    {
      if (_entity == this->model)
        return true;
      auto parent = _ecm.Component<components::ParentEntity>(_entity);
      if (!parent)
        return false;
      _entity = parent->Data();
    }
    return false;
  }

  // ---------------------------------------------------------------------
  private: void Register(EntityComponentManager &_ecm, const Entity &_coll,
                         const sdf::Collision &_collision)
  {
    const auto explicitFriction = ExplicitFriction(_collision);
    if (this->InModel(_ecm, _coll))
    {
      this->managed[_coll] = explicitFriction.value_or(this->ground);
      // Ask the Physics system to route this collision's contacts through
      // the CollectContactSurfaceProperties event. Physics removes the
      // component again once it has processed it.
      _ecm.CreateComponent(_coll,
          components::EnableContactSurfaceCustomization(true));
    }
    else
    {
      this->others[_coll] = explicitFriction.value_or(Coeffs{});
    }
  }

  // ---------------------------------------------------------------------
  private: template <typename Params>
  void OnContact(const Entity &_c1, const Entity &_c2, Params &_params) const
  {
    const auto m1 = this->managed.find(_c1);
    const auto m2 = this->managed.find(_c2);
    if (m1 == this->managed.end() && m2 == this->managed.end())
      return;

    const Coeffs a = (m1 != this->managed.end()) ? m1->second : this->Other(_c1);
    const Coeffs b = (m2 != this->managed.end()) ? m2->second : this->Other(_c2);
    _params.frictionCoeff = std::min(a.mu, b.mu);
    _params.secondaryFrictionCoeff = std::min(a.mu2, b.mu2);
  }

  private: Coeffs Other(const Entity &_coll) const
  {
    const auto it = this->others.find(_coll);
    return it != this->others.end() ? it->second : Coeffs{};
  }

  private: Entity model{kNullEntity};
  private: Coeffs ground;
  /// Collisions of this model -> coefficients to use for them.
  private: std::unordered_map<Entity, Coeffs> managed;
  /// Every other collision -> its SDF friction (or DART's default of 1).
  private: std::unordered_map<Entity, Coeffs> others;
  private: gz::common::ConnectionPtr connection;
};
}  // namespace gazebo_terrain_generator

GZ_ADD_PLUGIN(gazebo_terrain_generator::GroundFriction,
              gz::sim::System,
              gazebo_terrain_generator::GroundFriction::ISystemConfigure,
              gazebo_terrain_generator::GroundFriction::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gazebo_terrain_generator::GroundFriction,
                    "gazebo_terrain_generator::GroundFriction")
