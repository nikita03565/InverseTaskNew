#ifndef INVERSE_TASK_TESTS
#define INVERSE_TASK_TESTS

#include "CppUnitTest.h"
#include "../InverseTaskNew/fanucModel.h"
//#include "../InverseTaskNew/fanucModel.cpp"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace InverseTaskTest
{
    TEST_CLASS(UnitTest1)
    {
     nikita::FanucModel model;
     std::array<std::array<double, 6>, 72> coordsCart
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
     bool compare(int num);
     public:
        TEST_METHOD(testMethod0);
        TEST_METHOD(testMethod1);
        TEST_METHOD(testMethod2);
        TEST_METHOD(testMethod3);
        TEST_METHOD(testMethod4);
        TEST_METHOD(testMethod5);
        TEST_METHOD(testMethod6);
        TEST_METHOD(testMethod7);
        TEST_METHOD(testMethod8);
        TEST_METHOD(testMethod9);
        TEST_METHOD(testMethod10);
        TEST_METHOD(testMethod11);
        TEST_METHOD(testMethod12);
        TEST_METHOD(testMethod13);
        TEST_METHOD(testMethod14);
        TEST_METHOD(testMethod15);
        TEST_METHOD(testMethod16);
        TEST_METHOD(testMethod17);
        TEST_METHOD(testMethod18);
        TEST_METHOD(testMethod19);
        TEST_METHOD(testMethod20);
        TEST_METHOD(testMethod21);
        TEST_METHOD(testMethod22);
        TEST_METHOD(testMethod23);
        TEST_METHOD(testMethod24);
        TEST_METHOD(testMethod25);
        TEST_METHOD(testMethod26);
        TEST_METHOD(testMethod27);
        TEST_METHOD(testMethod28);
        TEST_METHOD(testMethod29);
        TEST_METHOD(testMethod30);
        TEST_METHOD(testMethod31);
        TEST_METHOD(testMethod32);
        TEST_METHOD(testMethod33);
        TEST_METHOD(testMethod34);
        TEST_METHOD(testMethod35);
        TEST_METHOD(testMethod36);
        TEST_METHOD(testMethod37);
        TEST_METHOD(testMethod38);
        TEST_METHOD(testMethod39);
        TEST_METHOD(testMethod40);
        TEST_METHOD(testMethod41);
        TEST_METHOD(testMethod42);
        TEST_METHOD(testMethod43);
        TEST_METHOD(testMethod44);
        TEST_METHOD(testMethod45);
        TEST_METHOD(testMethod46);
        TEST_METHOD(testMethod47);
        TEST_METHOD(testMethod48);
        TEST_METHOD(testMethod49);
        TEST_METHOD(testMethod50);
        TEST_METHOD(testMethod51);
        TEST_METHOD(testMethod52);
        TEST_METHOD(testMethod53);
        TEST_METHOD(testMethod54);
        TEST_METHOD(testMethod55);
        TEST_METHOD(testMethod56);
        TEST_METHOD(testMethod57);
        TEST_METHOD(testMethod58);
        TEST_METHOD(testMethod59);
        TEST_METHOD(testMethod60);
        TEST_METHOD(testMethod61);
        TEST_METHOD(testMethod62);
        TEST_METHOD(testMethod63);  
        TEST_METHOD(testMethod64);
        TEST_METHOD(testMethod65);
        TEST_METHOD(testMethod66);
        TEST_METHOD(testMethod67);
        TEST_METHOD(testMethod68);
        TEST_METHOD(testMethod69);
        TEST_METHOD(testMethod70);
        TEST_METHOD(testMethod71);
    };
}

#endif //INVERSE_TASK_TESTS
