/* File SingleObjectTracker.h
*//**
*     @file SingleObjectTracker.h
*     @brief This file declares the single object tracker class 
*     
*     @author Guled Elmi, ggredelmi@gmail.com
*     Created 02/05/2021
*
*/
#ifndef SINGLE_OBJECT_TRACKER_H
#define SINGLE_OBJECT_TRACKER_H

namespace sotTracker
{
    class SingleObjectTracker
    {
        public:
            SingleObjectTracker(){};

            

        private:
            GaussianDensity density;
            double gating; // gating size in percentage
            unsigned int gating_size;
            double pruning_threshold;
            double merging_threshold;
            unsigned int max_hypothesis;

    }

}
#endif