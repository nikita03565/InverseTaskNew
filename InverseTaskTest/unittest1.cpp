#include "unittest1.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

bool InverseTaskTest::UnitTest1::compare(int num)
{
    std::array<double, 6> res = nikita::FanucModel::getCoordsFromMat(model.fanucForwardTask(coords[num]));
    cv::Mat resInv = model.inverseTask(res);
    bool* f = new bool[resInv.rows];
    for (int i = 0; i < resInv.rows; ++i)
    {
        f[i] = true;
        for (int j = 0; j < 6; ++j)
        {
            if (abs(resInv.at<double>(i, j) - coords[num][j]) > 1e-5)
            {
                f[i] = false;
            }
        }
    }
    bool finalF = false;
    for (int i = 0; i < resInv.rows; ++i)
    {
        finalF = f[i] || finalF;
    }
    return finalF;
}

void InverseTaskTest::UnitTest1::testMethod0()
{
    Assert::AreEqual(true, compare(0));
}

void InverseTaskTest::UnitTest1::testMethod1()
{
    Assert::AreEqual(true, compare(1));
}

void InverseTaskTest::UnitTest1::testMethod2()
{
    Assert::AreEqual(true, compare(2));
}

void InverseTaskTest::UnitTest1::testMethod3()
{
    Assert::AreEqual(true, compare(3));
}

void InverseTaskTest::UnitTest1::testMethod4()
{
    Assert::AreEqual(true, compare(4));
}

void InverseTaskTest::UnitTest1::testMethod5()
{
    Assert::AreEqual(true, compare(5));
}

void InverseTaskTest::UnitTest1::testMethod6()
{
    Assert::AreEqual(true, compare(6));
}

void InverseTaskTest::UnitTest1::testMethod7()
{
    Assert::AreEqual(true, compare(7));
}

void InverseTaskTest::UnitTest1::testMethod8()
{
    Assert::AreEqual(true, compare(8));
}

void InverseTaskTest::UnitTest1::testMethod9()
{
    Assert::AreEqual(true, compare(9));
}

void InverseTaskTest::UnitTest1::testMethod10()
{
    Assert::AreEqual(true, compare(10));
}

void InverseTaskTest::UnitTest1::testMethod11()
{
    Assert::AreEqual(true, compare(11));
}

void InverseTaskTest::UnitTest1::testMethod12()
{
    Assert::AreEqual(true, compare(12));
}

void InverseTaskTest::UnitTest1::testMethod13()
{
    Assert::AreEqual(true, compare(13));
}

void InverseTaskTest::UnitTest1::testMethod14()
{
    Assert::AreEqual(true, compare(14));
}

void InverseTaskTest::UnitTest1::testMethod15()
{
    Assert::AreEqual(true, compare(15));
}

void InverseTaskTest::UnitTest1::testMethod16()
{
    Assert::AreEqual(true, compare(16));
}

void InverseTaskTest::UnitTest1::testMethod17()
{
    Assert::AreEqual(true, compare(17));
}

void InverseTaskTest::UnitTest1::testMethod18()
{
    Assert::AreEqual(true, compare(18));
}

void InverseTaskTest::UnitTest1::testMethod19()
{
    Assert::AreEqual(true, compare(19));
}

void InverseTaskTest::UnitTest1::testMethod20()
{
    Assert::AreEqual(true, compare(20));
}

void InverseTaskTest::UnitTest1::testMethod21()
{
    Assert::AreEqual(true, compare(21));
}

void InverseTaskTest::UnitTest1::testMethod22()
{
    Assert::AreEqual(true, compare(22));
}

void InverseTaskTest::UnitTest1::testMethod23()
{
    Assert::AreEqual(true, compare(23));
}

void InverseTaskTest::UnitTest1::testMethod24()
{
    Assert::AreEqual(true, compare(24));
}

void InverseTaskTest::UnitTest1::testMethod25()
{
    Assert::AreEqual(true, compare(25));
}

void InverseTaskTest::UnitTest1::testMethod26()
{
    Assert::AreEqual(true, compare(26));
}

void InverseTaskTest::UnitTest1::testMethod27()
{
    Assert::AreEqual(true, compare(27));
}

void InverseTaskTest::UnitTest1::testMethod28()
{
    Assert::AreEqual(true, compare(28));
}

void InverseTaskTest::UnitTest1::testMethod29()
{
    Assert::AreEqual(true, compare(29));
}

void InverseTaskTest::UnitTest1::testMethod30()
{
    Assert::AreEqual(true, compare(30));
}

void InverseTaskTest::UnitTest1::testMethod31()
{
    Assert::AreEqual(true, compare(31));
}

void InverseTaskTest::UnitTest1::testMethod32()
{
    Assert::AreEqual(true, compare(32));
}

void InverseTaskTest::UnitTest1::testMethod33()
{
    Assert::AreEqual(true, compare(33));
}

void InverseTaskTest::UnitTest1::testMethod34()
{
    Assert::AreEqual(true, compare(34));
}

void InverseTaskTest::UnitTest1::testMethod35()
{
    Assert::AreEqual(true, compare(35));
}

void InverseTaskTest::UnitTest1::testMethod36()
{
    Assert::AreEqual(true, compare(36));
}

void InverseTaskTest::UnitTest1::testMethod37()
{
    Assert::AreEqual(true, compare(37));
}

void InverseTaskTest::UnitTest1::testMethod38()
{
    Assert::AreEqual(true, compare(38));
}

void InverseTaskTest::UnitTest1::testMethod39()
{
    Assert::AreEqual(true, compare(39));
}

void InverseTaskTest::UnitTest1::testMethod40()
{
    Assert::AreEqual(true, compare(40));
}

void InverseTaskTest::UnitTest1::testMethod41()
{
    Assert::AreEqual(true, compare(41));
}

void InverseTaskTest::UnitTest1::testMethod42()
{
    Assert::AreEqual(true, compare(42));
}

void InverseTaskTest::UnitTest1::testMethod43()
{
    Assert::AreEqual(true, compare(43));
}

void InverseTaskTest::UnitTest1::testMethod44()
{
    Assert::AreEqual(true, compare(44));
}

void InverseTaskTest::UnitTest1::testMethod45()
{
    Assert::AreEqual(true, compare(45));
}

void InverseTaskTest::UnitTest1::testMethod46()
{
    Assert::AreEqual(true, compare(46));
}

void InverseTaskTest::UnitTest1::testMethod47()
{
    Assert::AreEqual(true, compare(47));
}

void InverseTaskTest::UnitTest1::testMethod48()
{
    Assert::AreEqual(true, compare(48));
}

void InverseTaskTest::UnitTest1::testMethod49()
{
    Assert::AreEqual(true, compare(49));
}

void InverseTaskTest::UnitTest1::testMethod50()
{
    Assert::AreEqual(true, compare(50));
}

void InverseTaskTest::UnitTest1::testMethod51()
{
    Assert::AreEqual(true, compare(51));
}

void InverseTaskTest::UnitTest1::testMethod52()
{
    Assert::AreEqual(true, compare(52));
}

void InverseTaskTest::UnitTest1::testMethod53()
{
    Assert::AreEqual(true, compare(53));
}

void InverseTaskTest::UnitTest1::testMethod54()
{
    Assert::AreEqual(true, compare(54));
}

void InverseTaskTest::UnitTest1::testMethod55()
{
    Assert::AreEqual(true, compare(55));
}

void InverseTaskTest::UnitTest1::testMethod56()
{
    Assert::AreEqual(true, compare(56));
}

void InverseTaskTest::UnitTest1::testMethod57()
{
    Assert::AreEqual(true, compare(57));
}

void InverseTaskTest::UnitTest1::testMethod58()
{
    Assert::AreEqual(true, compare(58));
}

void InverseTaskTest::UnitTest1::testMethod59()
{
    Assert::AreEqual(true, compare(59));
}

void InverseTaskTest::UnitTest1::testMethod60()
{
    Assert::AreEqual(true, compare(60));
}

void InverseTaskTest::UnitTest1::testMethod61()
{
    Assert::AreEqual(true, compare(61));
}

void InverseTaskTest::UnitTest1::testMethod62()
{
    Assert::AreEqual(true, compare(62));
}

void InverseTaskTest::UnitTest1::testMethod63()
{
    Assert::AreEqual(true, compare(63));
}

void InverseTaskTest::UnitTest1::testMethod64()
{
    Assert::AreEqual(true, compare(64));
}

void InverseTaskTest::UnitTest1::testMethod65()
{
    Assert::AreEqual(true, compare(65));
}

void InverseTaskTest::UnitTest1::testMethod66()
{
    Assert::AreEqual(true, compare(66));
}

void InverseTaskTest::UnitTest1::testMethod67()
{
    Assert::AreEqual(true, compare(67));
}

void InverseTaskTest::UnitTest1::testMethod68()
{
    Assert::AreEqual(true, compare(68));
}

void InverseTaskTest::UnitTest1::testMethod69()
{
    Assert::AreEqual(true, compare(69));
}

void InverseTaskTest::UnitTest1::testMethod70()
{
    Assert::AreEqual(true, compare(70));
}

void InverseTaskTest::UnitTest1::testMethod71()
{
    Assert::AreEqual(true, compare(71));
}