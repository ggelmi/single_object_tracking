#ifndef MAP_H
#define MAP_H

class Map
{
    public:
        struct landmark
        {
            double x;
            double y;
            int id;
        };

        std::vector<landmark> landmark_list;
};

#endif