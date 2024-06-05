#include "my_motion_validator.h"

bool MyMotionValidator::checkMotion(const ob::State *s1, const ob::State *s2) const
{
    // 상태 유효성 검사
    if (!si_->isValid(s1) || !si_->isValid(s2)) {
        std::cout << "State validity check failed." << std::endl;
        return false;
    }

    // 상태 전이 검사
    bool result = isMotionValid_(s1, s2);
    std::cout << "Motion validity check result: " << result << std::endl;
    return result;
}

bool MyMotionValidator::checkMotion(const ob::State *s1, const ob::State *s2, std::pair<ob::State *, double> &lastValid) const
{
    // 상태 유효성 검사
    if (!si_->isValid(s1)) {
        std::cout << "Initial state validity check failed." << std::endl;
        return false;
    }

    if (!si_->isValid(s2)) {
        std::cout << "Goal state validity check failed." << std::endl;
        return false;
    }

    // 상태 전이 검사
    if (isMotionValid_(s1, s2)) {
        std::cout << "Motion validity check passed." << std::endl;
        return true;
    }

    // 상태 전이가 유효하지 않으면 detailed check 수행
    lastValid.second = 0.0;
    const int steps = 10;
    ob::State *test = si_->allocState();
    for (int i = 1; i < steps; ++i)
    {
        si_->getStateSpace()->interpolate(s1, s2, (double)i / steps, test);
        if (!si_->isValid(test))
        {
            lastValid.first = si_->cloneState(test);
            lastValid.second = (double)(i - 1) / (steps - 1);
            si_->freeState(test);
            std::cout << "Detailed motion validity check failed at step " << i << std::endl;
            return false;
        }
    }
    si_->freeState(test);
    lastValid.first = si_->cloneState(s2);
    lastValid.second = 1.0;
    std::cout << "Detailed motion validity check passed." << std::endl;
    return true;
}
