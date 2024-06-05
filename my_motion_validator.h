#ifndef MYMOTIONVALIDATOR_H
#define MYMOTIONVALIDATOR_H

#include <ompl/base/SpaceInformation.h>
namespace ob = ompl::base;

class MyMotionValidator : public ompl::base::MotionValidator
{
public:
    MyMotionValidator(const ob::SpaceInformationPtr &si, const std::function<bool(const ob::State *, const ob::State *)> &isMotionValid)
        : ob::MotionValidator(si), isMotionValid_(isMotionValid) {}

    bool checkMotion(const ob::State *s1, const ob::State *s2) const override;
    bool checkMotion(const ob::State *s1, const ob::State *s2, std::pair<ob::State *, double> &lastValid) const override;

private:
    std::function<bool(const ob::State *, const ob::State *)> isMotionValid_;
};

#endif // MYMOTIONVALIDATOR_H
