//
// Copyright (c) 2015 CNRS
// Authors: Pierre Fernbach
//
// This file is part of hpp-flecto
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

#ifndef HPP_FLECTO_FLEXIBLE_VALIDATION_HH
# define HPP_FLECTO_FLEXIBLE_VALIDATION_HH

# include <hpp/core/collision-validation-report.hh>
# include <hpp/core/config-validation.hh>
//#include <hpp/model/fwd.hh>

namespace hpp {
  namespace flecto {
        typedef core::Configuration_t Configuration_t;
        typedef core::ValidationReport ValidationReport;
        typedef core::CollisionObjectPtr_t CollisionObjectPtr_t;
        typedef core::DevicePtr_t DevicePtr_t;
        typedef core::CollisionPairs_t CollisionPairs_t;
        typedef core::JointPtr_t JointPtr_t;
        typedef core::CollisionValidationReport CollisionValidationReport;


      class HPP_CORE_DLLAPI FlexibleValidation : public core::ConfigValidation
      {

      typedef boost::shared_ptr <FlexibleValidation> FlexibleValidationPtr_t;
      public:
        static FlexibleValidationPtr_t create (const DevicePtr_t& robot);

        /// Compute whether the configuration is valid
        ///
        /// \param config the config to check for validity,
        /// \param throwIfInValid if true throw an exception if config is invalid.
        /// \return whether the whole config is valid.
        virtual bool validate (const Configuration_t& config,
                   bool throwIfInValid = false);

        /// Compute whether the configuration is valid
        ///
        /// \param config the config to check for validity,
        /// \retval validationReport report on validation. This parameter will
        ///         dynamically cast into CollisionValidationReport type,
        /// \param throwIfInValid if true throw an exception if config is invalid,
        /// \return whether the whole config is valid.
        virtual bool validate (const Configuration_t& config,
                   ValidationReport& validationReport,
                   bool throwIfInValid = false);

        /// Add an obstacle
        /// \param object obstacle added
        /// Store obstacle and build a collision pair with each body of the robot.
        virtual void addObstacle (const CollisionObjectPtr_t& object);

        /// Remove a collision pair between a joint and an obstacle
        /// \param the joint that holds the inner objects,
        /// \param the obstacle to remove.
        /// \notice collision configuration validation needs to know about
        /// obstacles. This virtual method does nothing for configuration
        /// validation methods that do not care about obstacles.
        virtual void removeObstacleFromJoint
      (const JointPtr_t& joint, const CollisionObjectPtr_t& obstacle);
      protected:
        FlexibleValidation (const DevicePtr_t& robot);
      private:
        DevicePtr_t robot_;
        CollisionPairs_t collisionPairs_;
        /// This member is used by the validate method that does not take a
        /// validation report as input to call the validate method that expects
        /// a validation report as input. This is not fully satisfactory, but
        /// I did not find a better solution.
        CollisionValidationReport unusedReport;


      }; // class flexibleValidation
  } // namespace flecto
} // namespace hpp

#endif
