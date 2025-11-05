#ifndef RMATH_H
#define RMATH_H

#include <eigen3/Eigen/Dense>
#include <dof.h>
#include <tuple>
#include <deque>

#define MATH_PI             3.141592653589793
#define MATH_D2R            1.745329251994330e-2
#define MATH_R2D            5.729577951308232e1
#define MATH_INCH2MM        25.40
#define MATH_MAVG_SIZE      500


typedef Eigen::Matrix<double, NO_OF_JOINT, 1> VectorJd;
typedef Eigen::Matrix<double, NO_OF_CARTE, 1> VectorCd;

enum FRAME_LIST{
    FRAME_JOINT = -1,
    FRAME_GLOBAL = 0,
    FRAME_LOCAL = 1,
    FRAME_USER = 2,
    FRAME_TARGET = 3,
    FRAME_NUMBERS
};

enum UNIT_LIST{
    UNIT_MM_DEG = 0,
    UNIT_INCH_DEG,
};

struct TARGET_INPUT{
    // bigger number between NO_OF_JOINT vs NO_OF_CARTE
    double  target_value[NO_OF_JOINT];    

    // -1 : Joint
    // 0  : Global
    // 1  : Local
    // 2~ : User
    int     target_frame;
    
    // 0  : mm, deg
    // 1  : inch, deg
    int     target_unit;    
};

struct SHIFT_INPUT{
    int             mode;
    TARGET_INPUT    value;
};

struct Circle_3P_RET{
    bool result;             // 성공 여부
    Eigen::Vector3d center;  // 원의 중심
    Eigen::Vector3d axis;    // 회전축 (법선)
    double radius;           // 반지름
    double angle12;          // p1->p2 회전각
    double angle23;          // p2->p3 회전각
    double angle13;          // p1->p3 회전각
};

struct TEST_SAVE_STRUCT{
    float   data[30];
};

struct CAPSULE_STRUCT{
    Eigen::Vector3d    p0;
    Eigen::Vector3d    p1;
    float      radius;
};

struct BOX_STRUCT{
    Eigen::Vector3d     box_center;
    Eigen::Matrix3d     box_rot;
    Eigen::Vector3d     box_half;
};

namespace rb_math {
    Eigen::Matrix3d RPY_to_R(Eigen::Vector3d rpy_deg);
    Eigen::Matrix3d RPY_to_R(double rx_deg, double ry_deg, double rz_deg);
    Eigen::Matrix3d Rx(double rad);
    Eigen::Matrix3d Ry(double rad);
    Eigen::Matrix3d Rz(double rad);
    Eigen::Matrix3d RCV_to_R(Eigen::Vector3d trcv);
    Eigen::Vector3d R_to_RPY(Eigen::Matrix3d inR);
    Eigen::Vector3d R_to_RCV(Eigen::Matrix3d tR);

    Eigen::Vector3d Blend_Pos(Eigen::Vector3d p1, Eigen::Vector3d p2, double alpha);
    Eigen::Vector3d Blend_Rcv(Eigen::Vector3d r1, Eigen::Vector3d r2, double alpha);
    Eigen::Matrix3d Blend_Rmat(Eigen::Matrix3d R1, Eigen::Matrix3d R2, double alpha);
    Eigen::Vector3d Blend_Euler(Eigen::Vector3d E1, Eigen::Vector3d E2, double alpha);
    double Blend_NUM(double a1, double a2, double alpha);

    VectorJd Blend_Joint(VectorJd j1, VectorJd j2, double alpha);
    VectorCd Blend_Carte(VectorCd c1, VectorCd c2, double alpha);


    Eigen::Vector3d get_P_3x1(VectorCd posture);
    Eigen::Matrix3d get_R_3x3(VectorCd posture);
    Eigen::Vector3d get_E_3x1(VectorCd posture);
    double get_REDUN_1x1(VectorCd posture);

    #if NO_OF_JOINT == 7
    VectorCd Make_C_from_PandR(Eigen::Vector3d p, Eigen::Matrix3d R, double A);
    #else
    VectorCd Make_C_from_PandR(Eigen::Vector3d p, Eigen::Matrix3d R);
    #endif

    TARGET_INPUT Make_Input_from_Vec3(Eigen::Vector3d t3x1);
    TARGET_INPUT Make_Input_Zero(int frame);

    std::array<float, NO_OF_CARTE> Convert_VectorCd_to_Array(VectorCd inpt_c);
    std::array<float, NO_OF_JOINT> Convert_VectorJd_to_Array(VectorJd inpt_j);

    double sign(double in);
    double saturation_L_and_U(double in, double low, double up);
    double saturation_Low(double in, double low);
    double saturation_Up(double in, double up);

    double return_small(double a, double b);
    double return_big(double a, double b);

    double filt_Zero(double val, double x1, double x2, double y1, double y2);
    double filt_Line(double val, double x1, double x2, double y1, double y2);
    double filt_Curve(double val, double x1, double x2, double y1, double y2);

    std::tuple<double, double, double> Calc_Trapizoidal_Non_Symetric(double tD, double max_V, double a1, double a3);
    std::tuple<double, double, double, double, double, double, double, double> Calc_Trapizoidal(double max_V, double max_A, double tD, double start_vel, double end_vel);
    std::tuple<double, double, double, double, double, double, double, double> Calc_Trapizoidal(double max_V, double max_A, double tD);

    Circle_3P_RET fit_CircleFrom3Points(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3);

    double get_Average_Number(const std::deque<double>& buffer);
    Eigen::Vector3d get_Average_Pvec(const std::deque<Eigen::Vector3d>& buffer);
    Eigen::Matrix3d get_Average_Rmat(const std::deque<Eigen::Matrix3d>& buffer);
    VectorCd get_Average_VectorCd(const std::deque<VectorCd>& buffer);

    double Calc_LPF_Alpha(double Sampling_Hz, double CutOff_Hz);
    double Calc_CutOff_Hz(double Sampleing_Hz, double Alpha_v);

    double ShortestDistance_Box_Box(BOX_STRUCT box_A, BOX_STRUCT box_B);
    double ShortestDistance_Cap_Box(const CAPSULE_STRUCT cap, const BOX_STRUCT box);
    double ShortestDistance_Cap_Cap(const CAPSULE_STRUCT s1, const CAPSULE_STRUCT s2);

    void TestSaveStart();
    void TestSavePush(TEST_SAVE_STRUCT line);
    void TestSaveDone(std::string f_name);

    class MovingAverage {
    public:
        explicit MovingAverage(size_t size, size_t recalibrate_interval = 1000)
            : window_size(size),
            short_window_size(size / 2),
            index(0),
            count(0),
            sum(0.0),
            sum_short(0.0),
            recalibrate_interval(recalibrate_interval),
            update_count(0),
            data_buff(size, 0.0f)
        {
        }

        struct Result {
            float avg_full;   // 전체 이동평균
            float avg_short;  // 최근 절반 구간 평균
        };

        Result filter(float value) {
            const double old = data_buff[index];
            data_buff[index] = value;

            // -----------------------------
            // ① 전체 평균 합계 갱신
            // -----------------------------
            sum -= old;
            sum += value;

            // -----------------------------
            // ② 단기 평균(최근 절반 구간) 합계 갱신
            // -----------------------------
            // short_window_size 구간 뒤의 값은 short sum에서 제외
            const size_t short_tail_index = (index + window_size - short_window_size) % window_size;
            const double old_short = (count >= short_window_size) ? data_buff[short_tail_index] : 0.0;
            sum_short -= old_short;
            sum_short += value;

            // -----------------------------
            // ③ 인덱스/카운터 갱신
            // -----------------------------
            index = (index + 1) % window_size;
            if (count < window_size) count++;

            // -----------------------------
            // ④ 주기적 재합산 (drift 보정)
            // -----------------------------
            if (++update_count >= recalibrate_interval) {
                sum = 0.0;
                sum_short = 0.0;
                for (size_t i = 0; i < count; ++i) {
                    sum += data_buff[i];
                    if (i >= (count > short_window_size ? count - short_window_size : 0))
                        sum_short += data_buff[i];
                }
                update_count = 0;
            }

            // -----------------------------
            // ⑤ 평균 계산
            // -----------------------------
            double avg_full = sum / static_cast<double>(count);
            double avg_short = (count < short_window_size)
                                ? avg_full
                                : sum_short / static_cast<double>(short_window_size);

            return {static_cast<float>(avg_full), static_cast<float>(avg_short)};
        }

        void reset() {
            std::fill(data_buff.begin(), data_buff.end(), 0.0f);
            index = 0;
            count = 0;
            sum = 0.0;
            sum_short = 0.0;
            update_count = 0;
        }

        size_t size() const { return window_size; }

    private:
        size_t window_size;          // 전체 윈도 크기
        size_t short_window_size;    // 단기 평균 윈도 크기
        size_t index;                // 현재 인덱스
        size_t count;                // 입력된 실제 데이터 수
        double sum;                  // 전체 합
        double sum_short;            // 단기 합
        size_t recalibrate_interval; // 재합산 주기
        size_t update_count;         // 보정 카운터
        std::vector<float> data_buff;
    };

    class sloper_f {
    public:
        sloper_f(float init_value, float t_gap_up, float t_gap_dw);

        float Update(float new_input);
    private:
        float output_value;
        float gap_up;
        float gap_dw;
    };

    class SignalFilter{
    private:
        unsigned char stableSignal;
        unsigned char lastSignal;
        int count;
        int threshold;
    public:
        SignalFilter(int initThreshold = 1)
            : stableSignal(0), lastSignal(0), count(0), threshold(initThreshold) {}

        unsigned char update(unsigned char newSignal){
            if(newSignal == lastSignal){
                count++;
                if(count >= 2000000){
                    count = 2000000;
                }
            }else{
                count = 1;
            }

            lastSignal = newSignal;

            if(count >= threshold && stableSignal != newSignal){
                stableSignal = newSignal;
            }

            return stableSignal;
        }

        unsigned char getStableSignal() const{
            return stableSignal;
        }

        void setThreshold(int newThreshold){
            if(newThreshold > 0){
                threshold = newThreshold;
            }
        }
    };
}
#endif // RMATH_H
