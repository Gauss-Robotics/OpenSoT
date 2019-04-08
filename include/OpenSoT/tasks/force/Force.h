/*
 * Copyright (C) 2019 Cogimon/Centauro
 * Authors: Enrico Mingo Hoffman
 * email:  enrico.mingo@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU Lesser General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef __TASKS_FORCE_H__
#define __TASKS_FORCE_H__

#include <OpenSoT/Task.h>
#include <OpenSoT/utils/Affine.h>
#include <OpenSoT/tasks/MinimizeVariable.h>
#include <OpenSoT/tasks/Aggregated.h>
#include <boost/make_shared.hpp>
#include <OpenSoT/utils/AutoStack.h>

namespace OpenSoT {
   namespace tasks {
       namespace force {

        class Wrench : public Task < Eigen::MatrixXd, Eigen::VectorXd > {
        public:
            typedef boost::shared_ptr<Wrench> Ptr;

            Wrench(const std::string& contact_name, AffineHelper wrench);

            bool setReference(const Eigen::VectorXd& ref);
            void getReference(Eigen::VectorXd& ref);

            const std::string& getContactName();
        private:
            std::string _contact_name;
            virtual void _update(const Eigen::VectorXd& x);
            OpenSoT::tasks::MinimizeVariable::Ptr _min_var;

            Eigen::VectorXd _tmp;

        };

        class Wrenches : public Task < Eigen::MatrixXd, Eigen::VectorXd > {
        public:
            typedef boost::shared_ptr<Wrenches> Ptr;

            Wrenches(const std::vector<std::string>& contact_name,
                     std::vector<AffineHelper> wrenches);

            Wrench::Ptr getWrenchTask(const std::string& contact_name);

        private:
            std::map<std::string, Wrench::Ptr> wrench_tasks;
            OpenSoT::tasks::Aggregated::Ptr _aggregated_task;
            virtual void _update(const Eigen::VectorXd& x);


        };

       }
   }
}

#endif