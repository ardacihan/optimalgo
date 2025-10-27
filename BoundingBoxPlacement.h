#ifndef BOUNDING_BOX_PLACEMENT_H
#define BOUNDING_BOX_PLACEMENT_H

#include <vector>
#include "RectanglePlacement.h"

class BoundingBoxPlacement {
public:
    std::vector<RectanglePlacement> placements;
    int L;

    bool checkOverlapWithOtherRectangles(const RectanglePlacement &new_r);
    bool checkPlacementsWithinBoundingBox(const RectanglePlacement &new_r);
    bool place_rectangle(const RectanglePlacement &new_r);
};

#endif