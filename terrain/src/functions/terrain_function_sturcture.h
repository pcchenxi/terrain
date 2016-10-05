#ifndef _TERRAIN_FUNCTION_STURCTURE_H_  
#define _TERRAIN_FUNCTION_STURCTURE_H_  

struct Feature
{
  float continuity_prob;
  float cross_section_prob;
  float histogram_prob;
  float normal;
  float sum;
  float intensity;
  float reformed_height;

  float map_x;
  float map_y;
  float map_z;

/////////////////////////////////////////////////
  // bool  is_selected;
  float radius;
  float mean_height;
  float mean_slope;
  float height_variance;
  float vertical_acc;
  float roughness;
  float max_height_diff;

  float cost;
};

#endif

