//
// Copyright (c) 2014 CNRS
// Authors: Pierre Fernbach
//
// This file is part of hpp-core
// hpp-core is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#include <hpp/fcl/collision.h>
#include <hpp/model/body.hh>
#include <hpp/model/collision-object.hh>
#include <hpp/model/configuration.hh>
#include <hpp/model/device.hh>
#include <hpp/flecto/flexible-validation.hh>
#include <hpp/core/collision-validation-report.hh>


namespace hpp {
  namespace flecto {
  using model::displayConfig;
  typedef model::JointConfiguration* JointConfigurationPtr_t;
  typedef boost::shared_ptr <FlexibleValidation> FlexibleValidationPtr_t;



  FlexibleValidationPtr_t FlexibleValidation::create
  (const DevicePtr_t& robot)
  {
    FlexibleValidation* ptr = new FlexibleValidation (robot);
    return FlexibleValidationPtr_t (ptr);
  }

  bool FlexibleValidation::validate (const Configuration_t& config,
                  bool throwIfInValid)
  {
    return validate (config, unusedReport, throwIfInValid);
  }

  bool FlexibleValidation::validate (const Configuration_t& config,
                  ValidationReport& validationReport,
                  bool throwIfInValid)
  {
    HPP_STATIC_CAST_REF_CHECK (CollisionValidationReport, validationReport);
    CollisionValidationReport& report =  static_cast <CollisionValidationReport&> (validationReport);
    robot_->currentConfiguration (config);
    robot_->computeForwardKinematics ();
    bool collision = false;
    fcl::CollisionRequest collisionRequest (1, false, false, 1, false, true,
                        fcl::GST_INDEP);
    fcl::CollisionResult collisionResult;

    /*for (CollisionPairs_t::const_iterator itCol = collisionPairs_.begin ();
     itCol != collisionPairs_.end (); ++itCol) {
      if (fcl::collide (itCol->first->fcl ().get (),
                itCol->second->fcl ().get (),
                collisionRequest, collisionResult) != 0) {
        report.object1 = itCol->first;
        report.object2 = itCol->second;
        collision = true;
        break;
      }
    }*/



    if (collision && throwIfInValid) {
      std::ostringstream oss ("Configuration in collision: ");
      oss << displayConfig (config);
      throw std::runtime_error (oss.str ());
        }
    return !collision;
  }

  void FlexibleValidation::addObstacle (const CollisionObjectPtr_t& object)
  {
   // using model::COLLISION;
    const model::JointVector_t& jv = robot_->getJointVector ();
    for (model::JointVector_t::const_iterator it = jv.begin (); it != jv.end ();
     ++it) {
      core::JointPtr_t joint = *it;
      const core::BodyPtr_t body = joint->linkedBody ();
      if (body) {
        const core::ObjectVector_t& bodyObjects = body->innerObjects (model::COLLISION);
        for (core::ObjectVector_t::const_iterator itInner = bodyObjects.begin ();
             itInner != bodyObjects.end (); ++itInner) {
          collisionPairs_.push_back (core::CollisionPair_t (*itInner, object));
        }
      }
    }
  }

  void FlexibleValidation::removeObstacleFromJoint
  (const JointPtr_t& joint, const CollisionObjectPtr_t& obstacle)
  {
    using model::COLLISION;
    core::BodyPtr_t body = joint->linkedBody ();
   /* if (body) {
      const core::ObjectVector_t& bodyObjects = body->innerObjects (COLLISION);
      for (core::ObjectVector_t::const_iterator itInner = bodyObjects.begin ();
           itInner != bodyObjects.end (); ++itInner) {
        core::CollisionPair_t colPair (*itInner, obstacle);
        collisionPairs_.remove (colPair);
      }
    }*/
  }

  FlexibleValidation::FlexibleValidation (const DevicePtr_t& robot) :
    robot_ (robot)
  {
    using model::COLLISION;
    typedef hpp::model::Device::CollisionPairs_t JointPairs_t;
    using model::ObjectVector_t;
    const JointPairs_t& jointPairs (robot->collisionPairs (COLLISION));
    // build collision pairs for internal objects
    for (JointPairs_t::const_iterator it = jointPairs.begin ();
     it != jointPairs.end (); ++it) {
      JointPtr_t j1 = it->first;
      JointPtr_t j2 = it->second;
      core::BodyPtr_t body1 = j1->linkedBody ();
      core::BodyPtr_t body2 = j2->linkedBody ();
      /*if (body1 && body2) {
        core::ObjectVector_t objects1 = body1->innerObjects (COLLISION);
        core::ObjectVector_t objects2 = body2->innerObjects (COLLISION);
        // Loop over pairs of inner objects of the bodies
        for (core::ObjectVector_t::const_iterator it1 = objects1.begin ();
             it1 != objects1.end (); ++it1) {
          for (core::ObjectVector_t::const_iterator it2 = objects2.begin ();
           it2 != objects2.end (); ++it2) {
            collisionPairs_.push_back (core::CollisionPair_t (*it1, *it2));
          }
        }
      }*/
    }
  }


  } // namespace flecto
}//namespace hpp
