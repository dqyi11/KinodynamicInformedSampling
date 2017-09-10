#ifndef LOAD_HERB_H_
#define LOAD_HERB_H_

#include <libherb/herb.hpp>

std::shared_ptr<herb::Herb> loadHerb()
{
    std::shared_ptr<herb::Herb> herb = std::make_shared<herb::Herb>(true);
    return herb;
}

double getHerbRightArmVelUpperLimit(std::shared_ptr<herb::Herb> herb, int idx)
{
  if(herb->getRightArm())
  {
      return herb->getRightArm()->getVelocityUpperLimit(idx);
  }
  return 0.0;
}

double getHerbRightArmVelLowerLimit(std::shared_ptr<herb::Herb> herb, int idx)
{
  if(herb->getRightArm())
  {
      return herb->getRightArm()->getVelocityLowerLimit(idx);
  }
  return 0.0;
}

double getHerbRightArmPosUpperLimit(std::shared_ptr<herb::Herb> herb, int idx)
{
  if(herb->getRightArm())
  {
      return herb->getRightArm()->getPositionUpperLimit(idx);
  }
  return 0.0;
}

double getHerbRightArmPosLowerLimit(std::shared_ptr<herb::Herb> herb, int idx)
{
  if(herb->getRightArm())
  {
      return herb->getRightArm()->getPositionLowerLimit(idx);
  }
  return 0.0;
}


#endif // LOAD_HERB_H_
