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

/////////////////////////////////////////////////
  // bool  is_selected;
  float radius;
  float mean_height;
  float mean_slope;
  float height_variance;
  float vertical_acc;
  float roughness;
  float max_height_diff;

};

#endif

