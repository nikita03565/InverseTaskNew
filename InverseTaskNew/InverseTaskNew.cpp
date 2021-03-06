//#include <opencv2/core.hpp>
#include <array>
#include <iostream>
//#include "poly34.h"
#include "fanucModel.h"

std::array<double, 3> anglesFromMat(const cv::Mat p6)
{
    std::array<double, 3> angleVector{};
    angleVector.at(0) = atan2(p6.at<double>(2, 1), p6.at<double>(2, 2));
    angleVector.at(1) = atan2(-p6.at<double>(2, 0),
        sqrt(p6.at<double>(2, 1) * p6.at<double>(2, 1) + p6.at<double>(2, 2) * p6.at<double>(2, 2)));
    angleVector.at(2) = atan2(p6.at<double>(1, 0), p6.at<double>(0, 0));
    return angleVector;
}

std::array<double, 6> getCoordsFromMat(cv::Mat transformMatrix)
{
    std::array<double, 3> wprAngles = anglesFromMat(transformMatrix);

    return { {
            transformMatrix.at<double>(0, 3), transformMatrix.at<double>(1, 3), transformMatrix.at<double>(2, 3),
            wprAngles.at(0) * 180.0 / nikita::FanucModel::PI, wprAngles.at(1) * 180.0 / nikita::FanucModel::PI, wprAngles.at(2) * 180.0 / nikita::FanucModel::PI
        } };
}

int main()
{
    nikita::FanucModel model;
    std::array<std::array<double, 6>, 72> coords
    { {
        { 40, 40, 40, 40, 40, 40 },
    { 40, 40, 40, 40, 40, -40 },
    { 40, 40, 40, 40, -40, 40 },
    { 40, 40, 40, 40, 40, -40 },
    { 40, 40, 40, -40, 40, 40 },
    { 40, 40, 40, -40, 40, -40 },
    { 40, 40, 40, -40, -40, 40 },
    { 40, 40, 40, -40, -40, -40 },

    { 40, 40, -40, 40, 40, 40 },
    { 40, 40, -40, 40, 40, -40 },
    { 40, 40, -40, 40, -40, 40 },
    { 40, 40, -40, 40, 40, -40 },
    { 40, 40, -40, -40, 40, 40 },
    { 40, 40, -40, -40, 40, -40 },
    { 40, 40, -40, -40, -40, 40 },
    { 40, 40, -40, -40, -40, -40 },

    { 40, -40, 40, 40, 40, 40 },
    { 40, -40, 40, 40, 40, -40 },
    { 40, -40, 40, 40, -40, 40 },
    { 40, -40, 40, 40, 40, -40 },
    { 40, -40, 40, -40, 40, 40 },
    { 40, -40, 40, -40, 40, -40 },
    { 40, -40, 40, -40, -40, 40 },
    { 40, -40, 40, -40, -40, -40 },

    { 40, -40, -40, 40, 40, 40 },
    { 40, -40, -40, 40, 40, -40 },
    { 40, -40, -40, 40, -40, 40 },
    { 40, -40, -40, 40, 40, -40 },
    { 40, -40, -40, -40, 40, 40 },
    { 40, -40, -40, -40, 40, -40 },
    { 40, -40, -40, -40, -40, 40 },
    { 40, -40, -40, -40, -40, -40 },

    { -40, 40, 40, 40, 40, 40 },
    { -40, 40, 40, 40, 40, -40 },
    { -40, 40, 40, 40, -40, 40 },
    { -40, 40, 40, 40, 40, -40 },
    { -40, 40, 40, -40, 40, 40 },
    { -40, 40, 40, -40, 40, -40 },
    { -40, 40, 40, -40, -40, 40 },
    { -40, 40, 40, -40, -40, -40 },

    { -40, 40, -40, 40, 40, 40 },
    { -40, 40, -40, 40, 40, -40 },
    { -40, 40, -40, 40, -40, 40 },
    { -40, 40, -40, 40, 40, -40 },
    { -40, 40, -40, -40, 40, 40 },
    { -40, 40, -40, -40, 40, -40 },
    { -40, 40, -40, -40, -40, 40 },
    { -40, 40, -40, -40, -40, -40 },

    { -40, -40, 40, 40, 40, 40 },
    { -40, -40, 40, 40, 40, -40 },
    { -40, -40, 40, 40, -40, 40 },
    { -40, -40, 40, 40, 40, -40 },
    { -40, -40, 40, -40, 40, 40 },
    { -40, -40, 40, -40, 40, -40 },
    { -40, -40, 40, -40, -40, 40 },
    { -40, -40, 40, -40, -40, -40 },

    { -40, -40, -40, 40, 40, 40 },
    { -40, -40, -40, 40, 40, -40 },
    { -40, -40, -40, 40, -40, 40 },
    { -40, -40, -40, 40, 40, -40 },
    { -40, -40, -40, -40, 40, 40 },
    { -40, -40, -40, -40, 40, -40 },
    { -40, -40, -40, -40, -40, 40 },
    { -40, -40, -40, -40, -40, -40 },

    { -40, -40, -40, 140, 40, 140 },
    { -40, -40, -40, 140, 40, -140 },
    { -40, -40, -40, 140, -40, 140 },
    { -40, -40, -40, 140, 40, -140 },
    { -40, -40, -40, -140, 40, 140 },
    { -40, -40, -40, -140, 40, -140 },
    { -40, -40, -40, -140, -40, 140 },
    { -40, -40, -40, -140, -40, -140 }
        } };
    for (int i = 0; i < coords.size(); ++i)
    {
        std::cout << i << ": " << coords[i][0] << ' ' << coords[i][1] << ' ' << coords[i][2] << ' ' <<
            coords[i][3] << ' ' << coords[i][4] << ' ' << coords[i][5] << '\n';

        std::array<double, 6> res = getCoordsFromMat(model.fanucForwardTask(coords[i]));
        //std::array<double, 6> res = getCoordsFromMat(model.fanucForwardTask({ -10.4332,-60.0605,33.7463,-7.2156,19.0713,-0.12345 }));
        for (int j = 0; j < 6; ++j)
            std::cout << res[j] << ' ';
        std::cout << '\n';
        cv::Mat resInv = model.inverseTask(res);
        std::cout << resInv << "\n\n";
        for (int m = 0; m < resInv.rows; ++m)
        {
        std::array<double, 6> inputtmp;
        for (int j = 0; j < 6; ++j)
            inputtmp[j] = resInv.at<double>(m, j);
        std::array<double, 6> restmp = getCoordsFromMat(model.fanucForwardTask({ inputtmp }));
        for (int k = 0; k < 6; ++k)
            std::cout << inputtmp[k] << ' ';
        std::cout << ": ";
        for (int k = 0; k < 6; ++k)
            std::cout << restmp[k] << ' ';
        std::cout << '\n';
        }
    }
    //res = getCoordsFromMat(model.fanucForwardTask({ -70.4332,0.0605,3.7463,0.2156,-20.0713,-0.12345 }));
    ////std::array<double, 6> res = getCoordsFromMat(model.fanucForwardTask({ -10.4332,-60.0605,33.7463,-7.2156,19.0713,-0.12345 }));
    //std::cout << "\n--------------------------\n";
    //for (int i = 0; i < 6; ++i)
    //    std::cout << res[i] << ' ';
    //std::cout << '\n';
    //resInv = inverseTask(res);
    //std::cout << resInv << '\n';
    //
    //for (int i = 0; i < resInv.rows; ++i)
    //{
    //    std::array<double, 6> inputtmp;
    //    for (int j = 0; j < 6; ++j)
    //        inputtmp[j] = resInv.at<double>(i, j);
    //    std::array<double, 6> restmp = getCoordsFromMat(model.fanucForwardTask({ inputtmp }));
    //    for (int k = 0; k < 6; ++k)
    //        std::cout << restmp[k] << ' ';
    //    std::cout << '\n';
    //}
    std::cin.get();
    return 0;
}
//
//#define PI 3.14159265
//
//template <typename T> int sgn(T val) {
//    return (T(0) < val) - (val < T(0));
//}
//
//cv::Mat qi(const double& alpha, const  double& q)
//{
//    cv::Mat tmp(3, 3, CV_64F);
//    tmp.at<double>(0, 0) = cos(q);
//    tmp.at<double>(0, 1) = -cos(alpha) * sin(q);
//    tmp.at<double>(0, 2) = sin(alpha) * sin(q);
//
//    tmp.at<double>(1, 0) = sin(q);
//    tmp.at<double>(1, 1) = cos(alpha) * cos(q);
//    tmp.at<double>(1, 2) = -sin(alpha) * cos(q);
//
//    tmp.at<double>(2, 0) = 0;
//    tmp.at<double>(2, 1) = sin(alpha);
//    tmp.at<double>(2, 2) = cos(alpha);
//
//    return tmp;
//}
//
//cv::Mat rotMatrix(const double w, const double p, const double r)
//{
//    cv::Mat mx(3, 3, CV_64F), my(3, 3, CV_64F), mz(3, 3, CV_64F);
//    mx.at<double>(0, 0) = 1;
//    mx.at<double>(0, 1) = mx.at<double>(0, 2) = mx.at<double>(1, 0) = mx.at<double>(2, 0) = 0;
//    mx.at<double>(1, 1) = mx.at<double>(2, 2) = cos(w);
//    mx.at<double>(1, 2) = -sin(w);
//    mx.at<double>(2, 1) = sin(w);
//
//    my.at<double>(1, 1) = 1;
//    my.at<double>(0, 1) = my.at<double>(1, 2) = my.at<double>(1, 0) = my.at<double>(2, 1) = 0;
//    my.at<double>(0, 0) = my.at<double>(2, 2) = cos(p);
//    my.at<double>(0, 2) = sin(p);
//    my.at<double>(2, 0) = -sin(p);
//
//    mz.at<double>(2, 2) = 1;
//    mz.at<double>(0, 2) = mz.at<double>(1, 2) = mz.at<double>(2, 0) = mz.at<double>(2, 1) = 0;
//    mz.at<double>(1, 1) = mz.at<double>(0, 0) = cos(r);
//    mz.at<double>(0, 1) = -sin(r);
//    mz.at<double>(1, 0) = sin(r);
//    return mz * my * mx;
//}
//
//struct DhParameters
//{
//    double _dParam;
//    double _qParam;
//    double _aParam;
//    double _alphaParam;
//    DhParameters(const double d, const double q, const double a, const double alpha) : _dParam(d), _qParam(q), _aParam(a),
//        _alphaParam(alpha)
//    {
//    }
//    
//};
//
//void wrist(const cv::Mat p6, double& xc, double& yc, double& zc)
//{
//    xc = p6.at<double>(0, 3) - p6.at<double>(0, 2) * 100.;
//    yc = p6.at<double>(1, 3) - p6.at<double>(1, 2) * 100.;
//    zc = p6.at<double>(2, 3) - p6.at<double>(2, 2) * 100.;
//}
////spherical wrist coordinates: homepos 985, 0, 1040; 
//cv::Mat inverseTask(const std::array<double, 6>& coordIn)
//{
//    std::vector<DhParameters> param
//    {
//        {DhParameters(0, 0, 150, PI / 2)},
//        {DhParameters(0, 0, 790, 0)},
//        {DhParameters(0, 0, 250, PI / 2)},
//        {DhParameters(835, 0, 0, -PI / 2)},
//        {DhParameters(0, 0, 0, PI / 2)},
//        {DhParameters(100, 0, 0, 0)},
//        {DhParameters(130, PI / 2, -90, 0)},
//        {DhParameters(-190, 0, 0, 0)}
//    };
//
//    cv::Mat rot = rotMatrix(coordIn[3] / 180.0 * PI, coordIn[4] / 180.0 * PI, coordIn[5] / 180.0 * PI);
//    std::array<double, 6> coord{ {
//        coordIn[0] - rot.at<double>(0, 2) * 100.0,
//        coordIn[1] - rot.at<double>(1, 2) * 100.0,
//        coordIn[2] - rot.at<double>(2, 2) * 100.0,
//        coordIn[3],
//        coordIn[4],
//        coordIn[5]
//    } };
//
//    /*for (int i = 0; i < 6; ++i)
//        std::cout << coord[i] << ' ';
//    std::cout << '\n';*/
//
//    const double a = 2 * param[0]._aParam * coord[0];
//    const double b = 2 * param[0]._aParam * coord[1];
//    const double c = 2 * param[1]._aParam * param[2]._aParam - 2 * param[1]._dParam * param[3]._dParam *
//        sin(param[1]._alphaParam) * sin(param[2]._alphaParam);
//    const double d = 2 * param[2]._aParam * param[1]._dParam * sin(param[1]._alphaParam) + 2 * param[1]._aParam * param[3]._dParam
//        * sin(param[2]._alphaParam);
//    const double e = param[1]._aParam * param[1]._aParam + param[2]._aParam * param[2]._aParam + param[1]._dParam *
//        param[1]._dParam + param[2]._dParam * param[2]._dParam + param[3]._dParam * param[3]._dParam -
//        param[0]._aParam * param[0]._aParam - coord[0] * coord[0] - coord[1] * coord[1] -
//        (coord[2] - param[0]._dParam) * (coord[2] - param[0]._dParam) + 2 *
//        param[1]._dParam * param[2]._dParam * cos(param[1]._alphaParam) + 2 * param[1]._dParam * param[3]._dParam *
//        cos(param[1]._alphaParam) * cos(param[2]._alphaParam) + 2 * param[2]._dParam * param[3]._dParam * cos(param[2]._alphaParam);
//    const double f = coord[1] * sin(param[0]._alphaParam);
//    const double g = -coord[0] * sin(param[0]._alphaParam);
//    const double h = -param[3]._dParam * sin(param[1]._alphaParam) * sin(param[2]._alphaParam);
//    const double i = param[2]._aParam * sin(param[1]._alphaParam);
//    const double j = param[1]._dParam + param[2]._dParam * cos(param[1]._alphaParam) + param[3]._dParam *
//        cos(param[1]._alphaParam) * cos(param[2]._alphaParam) - (coord[2] - param[0]._dParam) *
//        cos(param[0]._alphaParam);
//    const double r = 4. * param[0]._aParam * param[0]._aParam * (j - h) * (j - h) + sin(param[0]._alphaParam) *
//        sin(param[0]._alphaParam) * (e - c) * (e - c)
//        - 4. * param[0]._aParam * param[0]._aParam * sin(param[0]._alphaParam) * sin(param[0]._alphaParam)
//        * (coord[0] * coord[0] + coord[1] * coord[1]);
//    const double s = 4. * (4. * param[0]._aParam * param[0]._aParam * i * (j - h) + sin(param[0]._alphaParam) *
//        sin(param[0]._alphaParam) * d *
//        (e - c));
//    const double t = 2. * (4. * param[0]._aParam * param[0]._aParam * (j * j - h * h + 2 * i * i) +
//        sin(param[0]._alphaParam) * sin(param[0]._alphaParam)
//        * (e * e - c * c + 2 * d * d) - 4. * param[0]._aParam * param[0]._aParam *
//        sin(param[0]._alphaParam) * sin(param[0]._alphaParam) *
//        (coord[0] * coord[0] + coord[1] * coord[1]));
//    const double u = 4. * (4. * param[0]._aParam * param[0]._aParam * i * (j + h) +
//        sin(param[0]._alphaParam) * sin(param[0]._alphaParam) * d * (e + c));
//    const double v = 4. * param[0]._aParam * param[0]._aParam * (h + j) * (h + j) + sin(param[0]._alphaParam) *
//        sin(param[0]._alphaParam) *
//        (e + c) * (e + c) - 4. * param[0]._aParam * param[0]._aParam * sin(param[0]._alphaParam) *
//        sin(param[0]._alphaParam) *
//        (coord[0] * coord[0] + coord[1] * coord[1]);
//
//    double x[4];
//    const int numberOfRoots = SolveP4(x, s / r, t / r, u / r, v / r);
//
//    if (numberOfRoots != 2 && numberOfRoots != 4)
//    {
//        std::cout << "\nsomething is wrong with roots of equatation\n";
//        return cv::Mat();
//    }
//
//    cv::Mat theta(numberOfRoots, 3, cv::DataType<double>::type);
//
//    for (int it = 0; it < numberOfRoots; ++it)
//    {
//        theta.at<double>(it, 2) = 2. * atan(x[it]);
//    }
//
//    double costheta, sintheta;
//    for (int it = 0; it < numberOfRoots; ++it)
//    {
//        costheta = (-g * (c * cos(theta.at<double>(it, 2)) + d * sin(theta.at<double>(it, 2)) + e) +
//            b * (h * cos(theta.at<double>(it, 2)) + i * sin(theta.at<double>(it, 2) + j))) / (a * g - f * b);
//        sintheta = (f * (c * cos(theta.at<double>(it, 2)) + d * sin(theta.at<double>(it, 2)) + e) -
//            a * (h * cos(theta.at<double>(it, 2)) + i * sin(theta.at<double>(it, 2) + j))) / (a * g - f * b);
//        if (sintheta >= 0)
//        {
//            theta.at<double>(it, 0) = acos(costheta);
//        }
//        else
//        {
//            theta.at<double>(it, 0) = -acos(costheta);
//        }
//    }
//
//    for (int it = 0; it < numberOfRoots; ++it)
//    {
//        const double a11 = param[1]._aParam + param[2]._aParam * cos(theta.at<double>(it, 2)) +
//            param[3]._dParam * sin(param[2]._alphaParam) * sin(theta.at<double>(it, 2));
//        const double a12 = -param[2]._aParam * cos(param[1]._alphaParam) *
//            sin(theta.at<double>(it, 2)) + param[2]._dParam * sin(param[1]._alphaParam) + param[3]._dParam *
//            sin(param[2]._alphaParam) * cos(param[1]._alphaParam) * cos(theta.at<double>(it, 2)) +
//            param[3]._dParam * sin(param[1]._alphaParam) * cos(param[2]._alphaParam);
//        costheta = (a11 * (coord[0] * cos(theta.at<double>(it, 0)) + coord[1] * sin(theta.at<double>(it, 0)) - param[0]._aParam)
//            - a12 * (-coord[0] * cos(param[0]._alphaParam) * sin(theta.at<double>(it, 0)) + coord[1] *
//                cos(param[0]._alphaParam) * cos(theta.at<double>(it, 0)) + (coord[2] - param[0]._dParam) * sin(param[0]._alphaParam)))
//            / (a11 * a11 + a12 * a12);
//        sintheta = (a12 * (coord[0] * cos(theta.at<double>(it, 0)) + coord[1] * sin(theta.at<double>(it, 0)) - param[0]._aParam) + a11 *
//            (-coord[0] * cos(param[0]._alphaParam) * sin(theta.at<double>(it, 0)) + coord[1] *
//                cos(param[0]._alphaParam) *
//                cos(theta.at<double>(it, 0)) + (coord[2] - param[0]._dParam) * sin(param[0]._alphaParam))) / (a11 * a11 + a12 * a12);
//        if (sintheta >= 0)
//        {
//            theta.at<double>(it, 1) = acos(costheta);
//        }
//        else
//        {
//            theta.at<double>(it, 1) = -acos(costheta);
//        }
//    }
//
//    for (int it = 0; it < numberOfRoots; ++it)
//    {
//        theta.at<double>(it, 1) = -theta.at<double>(it, 1) + PI / 2;
//        theta.at<double>(it, 2) -= theta.at<double>(it, 1);
//    }
//    //std::cout << "theta:\n" << theta * 180.0 / PI << '\n';
//    std::vector<int> ind;
//
//    for (int it = 0; it < numberOfRoots; ++it)
//    {
//        bool isOk = true;
//
//        if (abs(theta.at<double>(it, 0)) > 170. / 180. * PI)
//        {
//            isOk = false;
//        }
//        if (theta.at<double>(it, 1) > 90. / 180. * PI || theta.at<double>(it, 1) < -70 / 180. * PI)
//        {
//            isOk = false;
//        }
//        if (theta.at<double>(it, 2) > 200. / 180. * PI || theta.at<double>(it, 2) < -70 / 180. * PI)
//        {
//            isOk = false;
//        }
//
//        if (!std::isnan(theta.at<double>(it, 1)) && isOk)
//        {
//            ind.emplace_back(it);
//        }
//    }
//    cv::Mat thetaRes(ind.size(), 6, cv::DataType<double>::type);
//    for (int it = 0; it < ind.size(); ++it)
//    {
//        thetaRes.at<double>(it, 0) = theta.at<double>(ind[it], 0);
//        thetaRes.at<double>(it, 1) = theta.at<double>(ind[it], 1);
//        thetaRes.at<double>(it, 2) = theta.at<double>(ind[it], 2);
//        thetaRes.at<double>(it, 3) = 0;
//        thetaRes.at<double>(it, 4) = 0;
//        thetaRes.at<double>(it, 5) = 0;
//    }
//    //std::cout << "thetaRes:\n" << thetaRes * 180.0 / PI << '\n';
//    cv::Mat thetaPrefinal(thetaRes.rows * 2, 6, cv::DataType<double>::type);
//    //cv::Mat thetaPrefinal(thetaRes.rows, 6, cv::DataType<double>::type);
//    for (int it = 0; it < thetaRes.rows; ++it)
//    {
//        cv::Mat r36(3, 3, cv::DataType<double>::type), r03(3, 3, cv::DataType<double>::type);
//        std::array<double, 6> q;
//        q[0] = thetaRes.at<double>(it, 0);
//        q[1] = -thetaRes.at<double>(it, 1) + PI / 2;
//        q[2] = thetaRes.at<double>(it, 2) + thetaRes.at<double>(it, 1);
//
//        r03 = qi(param[0]._alphaParam, q[0]) * qi(param[1]._alphaParam, q[1]) * qi(param[2]._alphaParam, q[2]);
//        r36 = r03.inv() * rotMatrix(coord[3] / 180. * PI, coord[4] / 180. * PI, coord[5] / 180. * PI);
//        //std::cout << "r36:\n" << r36 << '\n';
//        /*thetaPrefinal.at<double>(it, 0) = thetaRes.at<double>(it, 0);
//        thetaPrefinal.at<double>(it, 1) = thetaRes.at<double>(it, 1);
//        thetaPrefinal.at<double>(it, 2) = thetaRes.at<double>(it, 2);*/
//        for (int zt = 0; zt < 2; ++zt)
//        {
//            thetaPrefinal.at<double>(it * 2 + zt, 0) = thetaRes.at<double>(it, 0);
//            thetaPrefinal.at<double>(it * 2 + zt, 1) = thetaRes.at<double>(it, 1);
//            thetaPrefinal.at<double>(it * 2 + zt, 2) = thetaRes.at<double>(it, 2);
//        }
//
//        thetaPrefinal.at<double>(it * 2, 3) = atan2(r36.at<double>(1, 2), r36.at<double>(0, 2));
//        thetaPrefinal.at<double>(it * 2, 4) = atan2( r36.at<double>(1, 2) / sin(thetaPrefinal.at<double>(it * 2, 3)), r36.at<double>(2, 2));
//        thetaPrefinal.at<double>(it * 2, 5) = atan2(r36.at<double>(2, 1), -r36.at<double>(2, 0));
//        
//        thetaPrefinal.at<double>(it * 2 + 1, 3) = PI + atan2(r36.at<double>(1, 2), r36.at<double>(0, 2));
//        thetaPrefinal.at<double>(it * 2 + 1, 4) = -atan2(r36.at<double>(1, 2) / sin(thetaPrefinal.at<double>(it * 2 + 1, 3)), r36.at<double>(2, 2));
//        thetaPrefinal.at<double>(it * 2 + 1, 5) = PI + atan2(r36.at<double>(2, 1), -r36.at<double>(2, 0));
//        if (abs(r36.at<double>(2, 2)) < 1e-6)
//        {
//            thetaPrefinal.at<double>(it * 2, 4) = PI / 2.0;
//            thetaPrefinal.at<double>(it * 2, 4) = -PI / 2.0;       
//        }
//        if (thetaPrefinal.at<double>(it * 2 + 1, 3) + 1e-6 > PI * 2.0)
//        {
//            thetaPrefinal.at<double>(it * 2 + 1, 3) -= PI * 2.0;
//        }
//        if (thetaPrefinal.at<double>(it * 2 + 1, 5) + 1e-6 > PI * 2.0)
//        {
//            thetaPrefinal.at<double>(it * 2 + 1, 5) -= PI * 2.0;
//        }
//    }
//    //std::cout << thetaPrefinal * 180.0 / PI << '\n';
//    std::vector<int> indFinal;
//    for (int it = 0; it < thetaPrefinal.rows; ++it)
//    {
//        bool isOk = true;
//
//        if (abs(thetaPrefinal.at<double>(it, 3)) > 210. / 180. * PI)
//        {
//            isOk = false;
//        }
//        if (abs(thetaPrefinal.at<double>(it, 4)) > 140. / 180. * PI)
//        {
//            isOk = false;
//        }
//        if (abs(thetaPrefinal.at<double>(it, 5)) > 270. / 180. * PI)
//        {
//            isOk = false;
//        }
//
//        if (isOk)
//        {
//            indFinal.push_back(it);
//        }
//    }
//
//    cv::Mat thetaFinal(indFinal.size(), 6, cv::DataType<double>::type);
//    for (int it = 0; it < indFinal.size(); ++it)
//    {
//        thetaFinal.at<double>(it, 0) = thetaPrefinal.at<double>(indFinal[it], 0);
//        thetaFinal.at<double>(it, 1) = thetaPrefinal.at<double>(indFinal[it], 1);
//        thetaFinal.at<double>(it, 2) = thetaPrefinal.at<double>(indFinal[it], 2);
//        thetaFinal.at<double>(it, 3) = -thetaPrefinal.at<double>(indFinal[it], 3);
//        thetaFinal.at<double>(it, 4) = thetaPrefinal.at<double>(indFinal[it], 4);
//        thetaFinal.at<double>(it, 5) = -thetaPrefinal.at<double>(indFinal[it], 5);
//    }
//
//    return thetaFinal * 180. / PI;
//}
//std::array<double, 6> chooseNearestPose(cv::Mat res, std::array<double, 6> prevPos) 
//{
//    if (!res.empty())
//    {
//        // std::cout << res << std::endl << std::endl;
//        std::vector<double> delta;
//        for (int j = 0; j < res.rows; ++j)
//        {
//            double deltaTmp = 0;
//            for (int t = 0; t < 6; ++t)
//            {
//                deltaTmp += abs(res.at<double>(j, t) - prevPos[t]);
//            }
//            delta.emplace_back(deltaTmp);
//        }
//        if (!delta.empty())
//        {
//            int num = 0;
//            double min = delta[0];
//            for (int j = 0; j < delta.size(); ++j)
//            {
//                if (delta[j] < min)
//                {
//                    min = delta[j];
//                    num = j;
//                }
//            }
//            if (min < 10.f)
//            {
//                return std::array<double, 6>{res.at<double>(num, 0), res.at<double>(num, 1), res.at<double>(num, 2), res.at<double>(num, 3), res.at<double>(num, 4), res.at<double>(num, 5) };
//            }
//        }
//        return prevPos;
//    }
//    return prevPos;
//}
//


//int main()
//{
//    auto qwe = rotMatrix(1, 2, 3);
//    std::cout << qwe << '\n';
//    double arr[16] = { 0.41198, -0.83374, -0.36763,	0.00000,
//        -0.05873, -0.42692,	0.90238,	0.00000,
//        -0.90930, -0.35018, -0.22485,	0.00000,
//        0.00000,	0.00000,	0.00000,	1.00000 };
//    cv::Mat mat(4, 4, cv::DataType<double>::type, arr);
//    std::cout << mat << '\n';
//    auto res = getCoordsFromMat(mat);
//    for (auto i : res)
//    {
//        std::cout << i << ' ';
//    }
//    std::cin.get();
//    return 0;
//}
//wrong
//std::array<double, 6> res = getCoordsFromMat(model.fanucForwardTask({ -10.4332,-60.0605,-33.7463,-7.2156,-19.0713,-0.12345 }));
