#include "lvx_loc.h"

LVX_LOC::LVX_LOC(QObject *parent)
    : QObject{parent}
{
    lvx_tf.setIdentity();
    cur_tf.setIdentity();

    tree = new KD_TREE(3, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
}

LVX_LOC::~LVX_LOC()
{
    is_connected = false;

    if(grab_thread != NULL)
    {
        grab_flag = false;
        grab_thread->join();
        grab_thread = NULL;
    }

    if(a_thread != NULL)
    {
        a_flag = false;
        a_thread->join();
        a_thread = NULL;
    }
}

void LVX_LOC::init()
{
    lvx_tf = string_to_TF(config->LVX_TF);

    printf("[LVX] init\n");
}

void LVX_LOC::open()
{
    // check simulation mode
    if(config->USE_SIM)
    {
        printf("[LVX] simulation mode\n");
        return;
    }

    printf("[LVX] connect\n");

    if(grab_thread == NULL)
    {
        grab_flag = true;
        grab_thread = new std::thread(&LVX_LOC::grab_loop, this);
    }
}

void LVX_LOC::map_load(QString file_path)
{
    // check simulation mode
    if(config->USE_SIM)
    {
        printf("[LVX] simulation mode\n");
        return;
    }

    if(last_map_file_path == file_path)
    {
        printf("[LVX] map_load, alread loaded\n");
        return;
    }
    last_map_file_path = file_path;

    QFile file(file_path);
    if(file.exists())
    {
        load_thread = new std::thread(&LVX_LOC::load_func, this, file_path);
    }
}

void LVX_LOC::load_func(QString file_path)
{
    is_loaded = MAP_LOADING;
    printf("[LVX] map load, %s\n", file_path.toLocal8Bit().data());

    pdal::StageFactory factory;
    pdal::Stage *reader = factory.createStage("readers.las");
    if(reader == NULL)
    {
        is_loaded = MAP_NOT_LOADED;
        printf("[LVX] map read failed\n");
        return;
    }

    pdal::Options options;
    options.add("filename", file_path.toStdString());
    options.add("extra_dims", "NormalX=double,NormalY=double,NormalZ=double");
    reader->setOptions(options);

    pdal::PointTable point_table;
    reader->prepare(point_table);

    pdal::PointViewSet point_views = reader->execute(point_table);
    pdal::PointViewPtr point_view = *point_views.begin();

    std::vector<Eigen::Vector2d> pts_2d;
    std::vector<Eigen::Vector3d> pts;
    std::vector<Eigen::Vector3d> nor;
    std::vector<double> reflects;
    std::vector<cv::Vec3b> colors;
    for(pdal::PointId id = 0; id < point_view->size(); id++)
    {
        double x = point_view->getFieldAs<double>(pdal::Dimension::Id::X, id);
        double y = point_view->getFieldAs<double>(pdal::Dimension::Id::Y, id);
        double z = point_view->getFieldAs<double>(pdal::Dimension::Id::Z, id);

        double nx = point_view->getFieldAs<double>(pdal::Dimension::Id::NormalX, id);
        double ny = point_view->getFieldAs<double>(pdal::Dimension::Id::NormalY, id);
        double nz = point_view->getFieldAs<double>(pdal::Dimension::Id::NormalZ, id);

        double intensity = point_view->getFieldAs<double>(pdal::Dimension::Id::Intensity, id);

        uint8_t red = point_view->getFieldAs<uint8_t>(pdal::Dimension::Id::Red, id);
        uint8_t green = point_view->getFieldAs<uint8_t>(pdal::Dimension::Id::Green, id);
        uint8_t blue = point_view->getFieldAs<uint8_t>(pdal::Dimension::Id::Blue, id);

        pts_2d.push_back(Eigen::Vector2d(x, y));
        pts.push_back(Eigen::Vector3d(x, y, z));
        nor.push_back(Eigen::Vector3d(nx, ny, nz));
        reflects.push_back(intensity);
        colors.push_back(cv::Vec3b(blue, green, red));

        if(id%1000 == 0)
        {
            printf("[LVX] map load, %d/%d ..\n", (int)id, (int)point_view->size());
        }
    }

    // update
    mtx.lock();
    map_pts = pts;
    map_nor = nor;
    cloud.pts = pts;
    tree->buildIndex();
    printf("[LVX] tree build success\n");
    mtx.unlock();

    is_loaded = MAP_LOADED;
    std::cout << "[LVX] Loaded " << pts.size() << " points from " << file_path.toStdString() << std::endl;
}

void LVX_LOC::loc_start()
{
    if(is_connected == false)
    {
        printf("[LVX] lidar not connected\n");
        return;
    }

    if(is_loaded != MAP_LOADED)
    {
        printf("[LVX] map not loaded\n");
        return;
    }

    if(a_thread == NULL)
    {
        a_flag = true;
        a_thread = new std::thread(&LVX_LOC::a_loop, this);
    }
}

void LVX_LOC::loc_stop()
{
    if(a_thread != NULL)
    {
        a_flag = false;
        a_thread->join();
        a_thread = NULL;
    }
}

void LVX_LOC::set_cur_tf(Eigen::Matrix4d tf)
{
    mtx.lock();
    cur_tf = tf;
    mtx.unlock();
}

Eigen::Matrix4d LVX_LOC::get_cur_tf()
{
    mtx.lock();
    Eigen::Matrix4d res = cur_tf;
    mtx.unlock();

    return res;
}

IMU LVX_LOC::get_best_imu(double ref_t)
{
    mtx.lock();
    double min_dt = 99999999;
    IMU res = imu_storage.back();
    for(size_t p = 0; p < imu_storage.size(); p++)
    {
        double dt = std::abs(imu_storage[p].t - ref_t);
        if(dt < min_dt)
        {
            min_dt = dt;
            res = imu_storage[p];
        }
    }
    mtx.unlock();

    return res;
}

QString LVX_LOC::get_info_text()
{
    Eigen::Matrix4d _cur_tf = get_cur_tf();
    Eigen::Vector3d xi = TF_to_se2(_cur_tf);

    QString str;
    str.sprintf("[LVX_INFO]\nt:%.2f, imu:%d, fq:%d (%d)\npose: %.2f, %.2f, %.2f\nie: %.2f, ir: %.2f",
                (double)sensor_t, (int)imu_storage.size(), (int)frm_que.unsafe_size(), (int)frm_pts_num,
                xi[0], xi[1], xi[2]*R2D,
                (double)cur_tf_ie, (double)cur_tf_ir);
    return str;
}

void LVX_LOC::grab_loop()
{
    // Disable logger
    DisableLivoxSdkConsoleLogger();
    printf("[LVX] Disable debug message\n");

    // Init Livox SDK2
    QString path = QCoreApplication::applicationDirPath() + "/config/mid360_config.json";
    printf("[LVX] load, %s\n", path.toLocal8Bit().data());
    if(!LivoxLidarSdkInit(path.toLocal8Bit().data()))
    {
        printf("[LVX] livox Init Failed\n");
        LivoxLidarSdkUninit();
        return;
    }

    printf("[LVX] livox initialized\n");

    // set callbacks
    auto point_cloud_callback = [](const uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data)
    {
        // check
        LVX_LOC* lidar = static_cast<LVX_LOC*>(client_data);
        if(lidar == nullptr)
        {
            return;
        }

        if(data == nullptr)
        {
            return;
        }

        // parsing metadata
        uint64_t time_base = *reinterpret_cast<const uint64_t*>(data->timestamp); // nanosec
        uint16_t time_interval = data->time_interval/10; // 0.1us -> nanosec

        // parsing point cloud
        if(data->data_type == kLivoxLidarCartesianCoordinateHighData)
        {
            LivoxLidarCartesianHighRawPoint *p_point_data = reinterpret_cast<LivoxLidarCartesianHighRawPoint *>(data->data);
            if(lidar->offset_t == 0)
            {
                return;
            }

            std::vector<LVX_PT> pts;
            for(uint32_t i = 0; i < data->dot_num; i++)
            {
                Eigen::Vector3d P;
                P[0] = p_point_data[i].x/1000.0;
                P[1] = p_point_data[i].y/1000.0;
                P[2] = p_point_data[i].z/1000.0;

                double d = P.norm();
                if(d < lidar->config->LVX_MIN_RANGE || d > lidar->config->LVX_MAX_RANGE)
                {
                    continue;
                }

                //Eigen::Vector3d _P = lidar->lvx_tf.block(0,0,3,3)*P + lidar->lvx_tf.block(0,3,3,1);
                Eigen::Vector3d _P = lidar->lvx_tf.block(0,0,3,3)*P; // rotation only

                LVX_PT pt;
                pt.t = (time_base + i * time_interval)*N2S + lidar->offset_t; // nanosec to sec
                pt.x = _P[0];
                pt.y = _P[1];
                pt.z = _P[2];
                pt.reflect = p_point_data[i].reflectivity;
                pt.tag = p_point_data[i].tag;

                pts.push_back(pt);
            }

            // update
            lidar->pts_storage.insert(lidar->pts_storage.end(), pts.begin(), pts.end());
            if(lidar->pts_storage.size() >= 2 && (lidar->pts_storage.back().t - lidar->pts_storage.front().t) > lidar->config->LVX_FRM_DT)
            {
                // calc alpha
                const double t0 = lidar->pts_storage.front().t;
                const double t1 = lidar->pts_storage.back().t;
                for(size_t p = 0; p < lidar->pts_storage.size(); p++)
                {
                    double t = lidar->pts_storage[p].t;
                    double alpha = (t-t0)/(t1-t0);
                    lidar->pts_storage[p].alpha = alpha;
                }

                // make frame
                LVX_FRM frm;
                frm.t = t0;
                frm.pts = lidar->pts_storage;

                // set queue
                lidar->frm_que.push(frm);
                lidar->frm_pts_num = frm.pts.size();

                // for queue overflow
                if(lidar->frm_que.unsafe_size() > 10)
                {
                    LVX_FRM tmp;
                    lidar->frm_que.try_pop(tmp);
                }

                // clear
                lidar->pts_storage.clear();
                lidar->pts_storage.push_back(frm.pts.back());
            }
        }
    };

    auto imu_data_callback = [](const uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data)
    {
        // check
        LVX_LOC* lidar = static_cast<LVX_LOC*>(client_data);
        if(lidar == nullptr)
        {
            return;
        }

        if(data == nullptr)
        {
            return;
        }

        // parsing
        uint64_t time_base = *reinterpret_cast<const uint64_t*>(data->timestamp); // nanosec

        // time sync
        if(lidar->offset_t == 0 || lidar->is_sync)
        {
            lidar->is_sync = false;
            lidar->offset_t = get_time() - (time_base * N2S);
        }

        LivoxLidarImuRawPoint *p_point_data = reinterpret_cast<LivoxLidarImuRawPoint *>(data->data);

        Eigen::Vector3d acc_vec;
        acc_vec[0] = p_point_data->acc_x; // normalized gravity vector
        acc_vec[1] = p_point_data->acc_y;
        acc_vec[2] = p_point_data->acc_z;

        Eigen::Vector3d gyr_vec;
        gyr_vec[0] = p_point_data->gyro_x; // rad/s
        gyr_vec[1] = p_point_data->gyro_y; // rad/s
        gyr_vec[2] = p_point_data->gyro_z; // rad/s

        Eigen::Vector3d _acc_vec = lidar->lvx_tf.block(0,0,3,3)*acc_vec;
        Eigen::Vector3d _gyr_vec = lidar->lvx_tf.block(0,0,3,3)*gyr_vec;

        IMU imu;
        imu.t = time_base * N2S + lidar->offset_t;
        imu.acc_x = _acc_vec[0]*ACC_G; // m/s^2
        imu.acc_y = _acc_vec[1]*ACC_G;
        imu.acc_z = _acc_vec[2]*ACC_G;
        imu.gyr_x = _gyr_vec[0]; // rad/s
        imu.gyr_y = _gyr_vec[1];
        imu.gyr_z = _gyr_vec[2];

        //printf("[LVX] imu received, t: %f\n", imu.t);

        // get orientation using complementary filter
        double q0, q1, q2, q3;
        lidar->imu_filter.update(_acc_vec[0], _acc_vec[1], _acc_vec[2], _gyr_vec[0], _gyr_vec[1], _gyr_vec[2], 0.005); // 200hz
        lidar->imu_filter.getOrientation(q0, q1, q2, q3);
        Eigen::Matrix3d R = Eigen::Quaterniond(q0, q1, q2, q3).normalized().toRotationMatrix();
        Eigen::Vector3d r = Sophus::SO3d::fitToSO3(R).log();

        imu.rx = r[0];
        imu.ry = r[1];
        imu.rz = r[2];

        // update storage
        lidar->mtx.lock();
        if(lidar->offset_t != 0)
        {
            lidar->sensor_t = imu.t;
            lidar->imu_storage.push_back(imu);
            if(lidar->imu_storage.size() > 200)
            {
                lidar->imu_storage.erase(lidar->imu_storage.begin());
            }
        }
        lidar->mtx.unlock();
    };

    // Register callbacks
    SetLivoxLidarPointCloudCallBack(point_cloud_callback, this); // client_data is unused
    SetLivoxLidarImuDataCallback(imu_data_callback, this); // client_data is unused

    printf("[LVX] callback registered\n");

    // set flag
    is_connected = true;

    printf("[LVX] grab_loop start\n");
    while(grab_flag)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // uninit
    LivoxLidarSdkUninit();

    printf("[LVX] grab_loop stop\n");
}

void LVX_LOC::a_loop()
{
    IMU pre_imu;

    Eigen::Matrix4d offset_tf = Eigen::Matrix4d::Identity();
    offset_tf.block(0,3,3,1) = lvx_tf.block(0,3,3,1);

    Eigen::Matrix4d offset_tf_inv = offset_tf.inverse();
    Eigen::Matrix4d _cur_tf = get_cur_tf()*offset_tf;
    Eigen::Matrix4d _pre_tf = _cur_tf;

    printf("[LVX] a_loop start\n");
    while(a_flag)
    {
        LVX_FRM frm;
        if(frm_que.try_pop(frm))
        {
            if(is_loaded != MAP_LOADED)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            // initial guess
            double dx = (_pre_tf.inverse()*_cur_tf)(0,3);
            Eigen::Matrix4d G = _cur_tf*ZYX_to_TF(dx, 0, 0, 0, 0, 0);

            // imu
            IMU cur_imu = get_best_imu(frm.t);
            if(pre_imu.t != 0)
            {
                Eigen::Matrix3d R0 = Sophus::SO3d::exp(Sophus::Vector3d(pre_imu.rx, pre_imu.ry, pre_imu.rz)).matrix();
                Eigen::Matrix3d R1 = Sophus::SO3d::exp(Sophus::Vector3d(cur_imu.rx, cur_imu.ry, cur_imu.rz)).matrix();
                G.block(0,0,3,3) = G.block(0,0,3,3)*(R0.inverse()*R1);
            }

            // deskewing
            std::vector<Eigen::Vector3d> dsk;
            {
                IMU imu0 = get_best_imu(frm.pts.front().t);
                IMU imu1 = get_best_imu(frm.pts.back().t);

                Eigen::Matrix3d R0 = Sophus::SO3d::exp(Sophus::Vector3d(imu0.rx, imu0.ry, imu0.rz)).matrix();
                Eigen::Matrix3d R1 = Sophus::SO3d::exp(Sophus::Vector3d(imu1.rx, imu1.ry, imu1.rz)).matrix();
                Eigen::Matrix3d dR = R0.inverse()*R1;
                Eigen::Matrix4d dG = Eigen::Matrix4d::Identity();
                dG.block(0,0,3,3) = dR;

                for(size_t p = 0; p < frm.pts.size(); p++)
                {
                    Eigen::Matrix4d G_i = intp_tf(frm.pts[p].alpha, Eigen::Matrix4d::Identity(), dG);
                    Eigen::Vector3d P(frm.pts[p].x, frm.pts[p].y, frm.pts[p].z);
                    Eigen::Vector3d _P = G_i.block(0,0,3,3)*P + G_i.block(0,3,3,1);
                    dsk.push_back(_P);
                }
            }

            // icp
            double err = map_icp(dsk, G);

            // check ieir
            Eigen::Vector2d ieir = calc_ieir(dsk, G);

            // update
            cur_tf_t = frm.t;
            cur_tf_err = err;
            cur_tf_ie = ieir[0];
            cur_tf_ir = ieir[1];
            set_cur_tf(G*offset_tf_inv);

            // for next
            _pre_tf = _cur_tf;
            _cur_tf = G;
            pre_imu = cur_imu;

            // for que overflow
            frm_que.clear();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    printf("[LVX] a_loop stop\n");
}

Eigen::Vector2d LVX_LOC::calc_ieir(std::vector<Eigen::Vector3d>& pts, Eigen::Matrix4d& G)
{
    if(pts.size() < 100)
    {
        return Eigen::Vector2d(1.0, 0);
    }

    double err_sum = 0;
    int cnt = 0;
    int cnt_all = 0;
    for(size_t p = 0; p < pts.size(); p += 8)
    {
        cnt_all++;

        Eigen::Vector3d P = pts[p];
        Eigen::Vector3d _P = G.block(0,0,3,3)*P + G.block(0,3,3,1);
        std::vector<int> nn_idxs = knn_search_idx(_P, 1, config->LVX_SURFEL_RANGE);
        if(nn_idxs.size() == 0)
        {
            continue;
        }

        Eigen::Vector3d nn_pt = map_pts[nn_idxs[0]];
        double d = (nn_pt - _P).norm();
        if(d < config->LVX_INLIER_CHECK_DIST)
        {
            err_sum += d;
            cnt++;
        }
    }

    Eigen::Vector2d res;
    res[0] = err_sum/cnt;
    res[1] = (double)cnt/cnt_all;
    return res;
}

std::vector<int> LVX_LOC::knn_search_idx(Eigen::Vector3d center, int k, double radius)
{
    const double sq_radius = radius*radius;

    std::vector<unsigned int> res_idxs(k);
    std::vector<double> res_sq_dists(k);
    double query_pt[3] = {center[0], center[1], center[2]};

    tree->knnSearch(&query_pt[0], k, &res_idxs[0], &res_sq_dists[0]);

    std::vector<int> res;
    for(size_t p = 0; p < res_idxs.size(); p++)
    {
        double dx = cloud.pts[res_idxs[p]][0] - cloud.pts[res_idxs[0]][0];
        double dy = cloud.pts[res_idxs[p]][1] - cloud.pts[res_idxs[0]][1];
        double dz = cloud.pts[res_idxs[p]][2] - cloud.pts[res_idxs[0]][2];
        double sq_d = dx*dx + dy*dy + dz*dz;
        if(sq_d < sq_radius)
        {
            res.push_back(res_idxs[p]);
        }
    }
    return res;
}

double LVX_LOC::map_icp(std::vector<Eigen::Vector3d>& pts, Eigen::Matrix4d& G)
{
    // for processing time
    double t_st = get_time();

    // for random selection
    std::vector<int> idx_list;
    for(size_t p = 0; p < pts.size(); p++)
    {
        // set idxs
        idx_list.push_back(p);
    }

    // solution
    Eigen::Matrix4d _G = G;

    // optimization param
    const int max_iter = max_iter0;
    double lambda = lambda0;
    double first_err = 9999;
    double last_err = 9999;
    double convergence = 9999;
    int num_correspondence = 0;

    // for rmt
    double tm0 = 0;
    double tm1 = 0;

    // loop
    int iter = 0;
    for(iter = 0; iter < max_iter; iter++)
    {
        // shuffle
        std::shuffle(idx_list.begin(), idx_list.end(), std::default_random_engine());
        const int num_feature = std::min<int>(idx_list.size(), config->LVX_MAX_FEATURE_NUM);
        std::vector<int> surfel_cnt(3,0);

        // calc cost jacobian
        std::vector<double> costs;
        std::vector<COST_JACOBIAN> cj_set;
        for(size_t k = 0; k < idx_list.size(); k++)
        {
            // get index
            int i = idx_list[k];

            // local to global
            Eigen::Vector3d P = pts[i];
            Eigen::Vector3d P1 = _G.block(0,0,3,3)*P + _G.block(0,3,3,1);

            // calc correspondence
            const int _nn_num = config->LVX_SURFEL_NN_NUM;
            std::vector<int> nn_idxs = knn_search_idx(P1, _nn_num, config->LVX_SURFEL_RANGE);
            if((int)nn_idxs.size() != _nn_num)
            {
                continue;
            }

            // calc surfel
            Eigen::Vector3d N0(0,0,0);
            Eigen::Vector3d P0(0,0,0);
            for(size_t p = 0; p < nn_idxs.size(); p++)
            {
                int idx = nn_idxs[p];
                P0 += map_pts[idx];
                N0 += map_nor[idx];
            }

            P0 /= nn_idxs.size();
            N0 /= nn_idxs.size();
            N0.normalize();

            // balance filter
            int axis_idx = get_major_axis(N0);
            if(surfel_cnt[axis_idx] > num_feature*config->LVX_SURFEL_BALANCE)
            {
                continue;
            }

            // rmt
            double rmt = 1.0;
            if(iter >= 1)
            {
                rmt = tm1/tm0;
                if(rmt > 1.0)
                {
                    rmt = 1.0;
                }
            }

            // calc cost
            double cost = N0.dot(P1-P0);
            if(std::abs(cost) > config->LVX_COST_THRESHOLD || std::abs(cost) > rmt*std::abs(cost) + rmt_sigma)
            {
                continue;
            }

            // Compute Jacobian using Lie Algebra
            Eigen::Matrix<double, 3, 6> dP1_dxi; // Jacobian of SE(3) transformation
            dP1_dxi.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(); // Translation part
            dP1_dxi.block<3, 3>(0, 3) = -Sophus::SO3d::hat(P1);      // Rotation part

            // Cost Jacobian: N0^T * dP1/dxi
            Eigen::Matrix<double, 1, 6> jacobian = N0.transpose() * dP1_dxi;
            if(!jacobian.allFinite())
            {
                continue;
            }

            double J[6];
            J[0]  = jacobian(0,0);
            J[1]  = jacobian(0,1);
            J[2]  = jacobian(0,2);
            J[3]  = jacobian(0,3);
            J[4]  = jacobian(0,4);
            J[5]  = jacobian(0,5);

            // storing cost jacobian
            COST_JACOBIAN cj;
            cj.c = cost;
            cj.w = 1.0;
            memcpy(cj.J, J, sizeof(double)*12);

            cj_set.push_back(cj);
            costs.push_back(cost);

            // update surfel count
            surfel_cnt[axis_idx]++;

            // check num
            if((int)cj_set.size() == num_feature)
            {
                break;
            }
        }

        // num of correspondence
        num_correspondence = cj_set.size();
        if(num_correspondence < 100)
        {
            printf("[map_icp] not enough correspondences, iter: %d, num: %d!!\n", iter, num_correspondence);
            return 9999;
        }

        // calc mu
        std::nth_element(costs.begin(), costs.begin() + costs.size()/2, costs.end());
        double mu = costs[costs.size() / 2];

        // calc sigma
        std::vector<double> vars(costs.size());
        for(size_t p = 0; p < costs.size(); p++)
        {
            vars[p] = std::abs(costs[p] - mu);
        }

        std::nth_element(vars.begin(), vars.begin() + vars.size()/2, vars.end());
        double sigma = std::max(1.4826 * vars[vars.size()/2], sigma_eps);

        // calc weight
        for(size_t p = 0; p < cj_set.size(); p++)
        {
            double c = cj_set[p].c;
            double V0 = t_dist_v0;
            double w = (V0 + 1.0) / (V0 + ((c - mu) / sigma) * ((c - mu) / sigma));
            cj_set[p].w *= std::clamp(w, 0.01, 10.0);
        }

        // make matrix
        double _A[6*6] = { 0, };
        double _b[6] = { 0, };
        double err = 0;
        double err_cnt = 0;
        for(size_t p = 0; p < cj_set.size(); p++)
        {
            double c = cj_set[p].c;
            double* J = cj_set[p].J;
            double w = cj_set[p].w;

            // set tempolar matrix(col major)
            for(int y = 0; y < 6; y++)
            {
                for(int x = 0; x < 6; x++)
                {
                    _A[y * 6 + x] += w * J[y] * J[x];
                }
                _b[y] += w * c * J[y];
            }

            // error
            err += std::abs(c);
            err_cnt += w;
        }
        err /= err_cnt;

        // set first error
        if(iter == 0)
        {
            first_err = err;
        }

        // solve
        Eigen::Matrix<double, 6, 6> A(_A);
        A += 1e-6*Eigen::Matrix<double, 6, 6>::Identity();
        Eigen::Matrix<double, 6, 1> b(_b);
        Eigen::Matrix<double, 6, 6> diag_A = A.diagonal().asDiagonal();
        Eigen::Matrix<double, 6, 1> X = (-(A + lambda * diag_A)).ldlt().solve(b);

        // lambda update
        double err_ratio = (last_err-err)/last_err;
        if(err_ratio > 0.1)
        {
            //lambda *= lambda_dec;
            lambda = std::max(lambda * lambda_dec, 1e-6);
        }
        else if(err_ratio < 0.01)
        {
            //lambda *= lambda_inc;
            lambda = std::min(lambda * lambda_inc, 1e6);
        }
        last_err = err;

        // pose update
        Sophus::Vector6d xi;
        xi[0] = X(0, 0); // tx
        xi[1] = X(1, 0); // ty
        xi[2] = X(2, 0); // tz
        xi[3] = X(3, 0); // rx
        xi[4] = X(4, 0); // ry
        xi[5] = X(5, 0); // rz

        _G = Sophus::SE3d::exp(xi).matrix()*_G;

        // for rmt
        tm0 = tm1;
        tm1 = (_G.block(0,3,3,1).norm() + _G.block(0,3,3,1).norm())/2;

        // convergence check
        convergence = X.cwiseAbs().maxCoeff();
        if(convergence < 1e-7)
        {
            break;
        }
    }

    // update
    G = _G;

    // for debug
    //printf("[LVX] map_icp, i:%d, n:%d, e:%f->%f, c:%e, dt:%.3f\n", iter, num_correspondence, first_err, last_err, convergence, get_time()-t_st);

    return last_err;
}

