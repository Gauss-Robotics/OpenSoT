/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi
 * email:  alessio.rocchi@iit.it
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

#include <wb_sot/bounds/velocity/Aggregated.h>

#include <yarp/math/Math.h>
#include <assert.h>
#include <limits>

using namespace wb_sot::bounds::velocity;
using namespace yarp::math;

Aggregated::Aggregated(const std::list< BoundType* >& bounds,
                       const unsigned int x_size,
                       const unsigned int aggregationPolicy) :
    Bounds(x_size), _bounds(bounds), _aggregationPolicy(aggregationPolicy)
{
    /* calling update to generate bounds */
    update(yarp::sig::Vector(x_size, 0.0));
}

void Aggregated::update(const yarp::sig::Vector& x) {

    /* resetting all internal data */
    _upperBound = yarp::sig::Vector(0);
    _lowerBound = yarp::sig::Vector(0);

    _Aeq = yarp::sig::Matrix(0,0);
    _beq = yarp::sig::Vector(0);

    _Aineq = yarp::sig::Matrix(0,0);
    _bUpperBound = yarp::sig::Vector(0);
    _bLowerBound = yarp::sig::Vector(0);

    /* iterating on all bounds.. */
    for(typename std::list< BoundType* >::iterator i = _bounds.begin();
        i != _bounds.end(); i++) {

        BoundType* b = *i;
        /* update bounds */
        b->update(x);

        yarp::sig::Vector boundUpperBound = b->getUpperBound();
        yarp::sig::Vector boundLowerBound = b->getLowerBound();

        yarp::sig::Matrix boundAeq = b->getAeq();
        yarp::sig::Vector boundbeq = b->getbeq();

        yarp::sig::Matrix boundAineq = b->getAineq();
        yarp::sig::Vector boundbUpperBound = b->getbUpperBound();
        yarp::sig::Vector boundbLowerBound = b->getbLowerBound();

        /* copying lowerBound, upperBound */
        if(boundUpperBound.size() != 0 ||
           boundLowerBound.size() != 0) {
            assert(boundUpperBound.size() == _x_size);
            assert(boundLowerBound.size() == _x_size);

            if(_upperBound.size() == 0 ||
               _lowerBound.size() == 0) { // first valid bounds found
                assert(_upperBound.size() == _lowerBound.size());
                _upperBound = boundUpperBound;
                _lowerBound = boundLowerBound;
            } else {
                for(unsigned int i = 0; i < _x_size; ++i) {
                    // compute the minimum between current and new upper bounds
                    _upperBound[i] = std::min(  _upperBound[i],
                                                boundUpperBound[i]);
                    // compute the maximum between current and new lower bounds
                    _lowerBound[i] = std::max(  _lowerBound[i],
                                                boundLowerBound[i]);
                }
            }
        }

        /* copying Aeq, beq */
        if( boundAeq.rows() != 0 ||
            boundbeq.size() != 0) {
            assert(boundAeq.rows() == boundbeq.size());
            /* when transforming equalities to inequalities,
                Aeq*x = beq becomes
                beq <= Aeq*x <= beq */
            if(_aggregationPolicy & EQUALITIES_TO_INEQUALITIES) {
                _Aineq = yarp::math::pile(_Aineq, boundAeq);
                _bUpperBound = yarp::math::cat(_bUpperBound,
                                               boundbeq);
                if(_aggregationPolicy & UNILATERAL_TO_BILATERAL) {
                    _bLowerBound = yarp::math::cat(_bLowerBound,
                                                   boundbeq);
                /* we want to have only unilateral constraints, so
                   beq <= Aeq*x <= beq becomes
                   -Aeq*x <= -beq && Aeq*x <= beq */
                } else {
                    _Aineq = yarp::math::pile(_Aineq, -1.0 * boundAeq);
                    _bUpperBound = yarp::math::cat(_bUpperBound,
                                                   -1.0 * boundbeq);
                }
            } else {
                _Aeq = yarp::math::pile(_Aeq, boundAeq);
                _beq = yarp::math::cat(_beq, boundbeq);
            }
        }

        /* copying Aineq, bUpperBound, bLowerBound*/
        if( boundAineq.rows() != 0 ||
            boundbUpperBound.size() != 0 ||
            boundbLowerBound.size() != 0) {

            /* if we need to transform all unilateral bounds to bilateral.. */
            if(_aggregationPolicy & UNILATERAL_TO_BILATERAL) {
                if(boundbUpperBound.size() == 0) {
                    boundbUpperBound.resize(boundAineq.rows(),
                                            std::numeric_limits<double>::infinity());
                    assert(boundAineq.rows() == boundbLowerBound.size());
                } else if(boundbLowerBound.size() == 0) {
                    assert(boundAineq.rows() == boundbUpperBound.size());
                    boundbUpperBound.resize(boundAineq.rows(),
                                            std::numeric_limits<double>::lowest());
                } else {
                    assert(boundAineq.rows() == boundbLowerBound.size());
                    assert(boundAineq.rows() == boundbUpperBound.size());
                }
            /* if we need to transform all bilateral bounds to unilateral.. */
            } else {
                /* we need to transform l < Ax into -Ax < -l */
                if(boundbUpperBound.size() == 0) {
                    boundAineq = -1.0 * boundAineq;
                    boundbLowerBound = -1.0 * boundbLowerBound;
                    assert(boundAineq.rows() == boundbLowerBound.size());
                } else if(boundbLowerBound.size() == 0) {
                    assert(boundAineq.rows() == boundbUpperBound.size());
                } else {
                    assert(boundAineq.rows() == boundbLowerBound.size());
                    assert(boundAineq.rows() == boundbUpperBound.size());
                    boundAineq = yarp::math::pile(boundAineq,
                                                  -1.0 * boundAineq);
                    boundbUpperBound = yarp::math::cat(boundbUpperBound,
                                                       -1.0 * boundbLowerBound);
                }
            }

            _Aineq = yarp::math::pile(_Aineq, boundAineq);
            _bUpperBound = yarp::math::cat(_bUpperBound, boundbUpperBound);
            /*  if using UNILATERAL_TO_BILATERAL we always have lower bounds,
                otherwise, we never have them */
            if(_aggregationPolicy & UNILATERAL_TO_BILATERAL)
                _bLowerBound = yarp::math::cat(_bLowerBound, boundbLowerBound);
        }
    }

    /* checking everything went fine */
    assert(_lowerBound.size() == 0 || _lowerBound.size() == _x_size);
    assert(_upperBound.size() == 0 || _upperBound.size() == _x_size);

    assert(_Aeq.rows() == _beq.size());
    assert(_Aeq.cols() == _x_size);

    assert(_Aineq.rows() == _bUpperBound.size());
    if(!(_aggregationPolicy & UNILATERAL_TO_BILATERAL))
        assert(_Aineq.rows() == _bLowerBound.size());
    assert(_Aineq.cols() == _x_size);
}
