#include <stdio.h>
#include <tchar.h>
#include "SerialClass.h"	// Library described above
#include <string>
#include <iostream>
#include "Control.h"
#include <array>
#include <math.h>

#define pi = 3.14159
void DynamicsTest(double Q1, double Q2, double Q3, double  Q4, double Q5v, double Q5h)
{
	std::array<double, 6> tau;
	double DQ1, DQ2, DQ3, DQ4, DQ5v, DQ5h, DDQ1, DDQ2, DDQ3, DDQ4, DDQ5v, DDQ5h;
	//Q1 = Q2 = Q3 = Q4 = Q5v = Q5h = 
	DQ1 = DQ2 = DQ3 = DQ4 = DQ5v = DQ5h = DDQ1 = DDQ2 = DDQ3 = DDQ4 = DDQ5v = DDQ5h = 0;
	double t2 = cos(Q1);
	double t3 = cos(Q2);
	double t4 = cos(Q3);
	double t5 = cos(Q4);
	double t6 = cos(Q5h);
	double t7 = cos(Q5v);
	double t8 = sin(Q1);
	double t9 = sin(Q2);
	double t10 = sin(Q3);
	double t11 = sin(Q4);
	double t12 = sin(Q5h);
	double t13 = sin(Q5v);
	double t14 = DQ1 * DQ1;
	double t15 = DQ2 * DQ2;
	double t16 = DQ3 * DQ3;
	double t17 = DQ4 * DQ4;
	double t18 = DQ5h * DQ5h;
	double t19 = DQ5v * DQ5v;
	double t20 = Q1 * 2.0;
	double t21 = Q1 * 3.0;
	double t22 = Q2 * 2.0;
	double t23 = Q1 * 4.0;
	double t24 = Q3 * 2.0;
	double t25 = Q1 * 5.0;
	double t26 = Q3 * 3.0;
	double t27 = Q4 * 2.0;
	double t28 = Q1 * 6.0;
	double t29 = Q3 * 4.0;
	double t30 = Q4 * 3.0;
	double t31 = Q3 * 5.0;
	double t32 = Q4 * 4.0;
	double t33 = Q4 * 5.0;
	double t34 = Q5h * 2.0;
	double t35 = Q5h * 3.0;
	double t36 = Q5h * 4.0;
	double t37 = Q5v * 2.0;
	double t38 = Q5v * 3.0;
	double t39 = Q5v * 4.0;
	double t40 = cos(t20);
	double t41 = cos(t21);
	double t42 = cos(t23);
	double t43 = cos(t24);
	double t44 = cos(t25);
	double t45 = cos(t26);
	double t46 = cos(t27);
	double t47 = cos(t29);
	double t48 = cos(t30);
	double t49 = cos(t32);
	double t50 = cos(t33);
	double t51 = cos(t34);
	double t52 = cos(t35);
	double t53 = cos(t36);
	double t54 = cos(t37);
	double t55 = cos(t38);
	double t56 = cos(t39);
	double t57 = t3 * t3;
	double t58 = t3 * t3 * t3;
	double t60 = t3 * t3 * t3 * t3 * t3;
	double t61 = sin(t20);
	double t62 = sin(t21);
	double t63 = sin(t22);
	double t64 = sin(t23);
	double t65 = sin(t24);
	double t66 = sin(t25);
	double t67 = sin(t26);
	double t68 = sin(t27);
	double t69 = sin(t29);
	double t70 = sin(t30);
	double t71 = sin(t32);
	double t72 = sin(t33);
	double t73 = sin(t34);
	double t74 = sin(t35);
	double t75 = sin(t36);
	double t76 = sin(t37);
	double t77 = sin(t38);
	double t78 = sin(t39);
	double t79 = t9 * t9 * t9;
	double t59 = t57 * t57;
	tau[0] = DDQ1 * 2.167526325145843E+8 - t2 * 1.5621642598155 + t8 * 1.006658660755E-1 - t14 * 1.527208334375E+9 - t40 * 1.3320989084E-2 - t41 * 4.99383704705E-2 - t42 * 1.9910282734E-2 - t61 * 3.7591660657E-2 - t62 * 5.86163957965E-2 + DDQ1 * t2 * 8.275948400064784E+8 + DDQ1 * t8 * 2.4990958500389E+8 + DDQ1 * t40 * 2.855738375008132E+8 + DDQ1 * t41 * 1.141876800005472E+8 + DDQ1 * t42 * 1.68161744567E-4 + DDQ1 * t44 * 3.3648732406E-5 - DDQ1 * t61 * 9.619994449967929E+8 - DDQ1 * t62 * 2.425044000072215E+7 + DDQ1 * t64 * 1.4595594943E-5 - DDQ1 * t66 * 1.97206894261E-4 - t2 * t14 * 1.91232753687111E+9 - t8 * t14 * 7.602519968814784E+8 - t14 * t40 * 2.081826574993586E+9 + t14 * t41 * 3.554370765603336E+8 - t14 * t42 * 4.496510062494162E+7 - t14 * t44 * 2.854551968848603E+7 - t14 * t61 * 3.270700950016263E+8 - t14 * t62 * 2.919739421891416E+8 + t14 * t64 * 1.339164999993274E+8 + t14 * t66 * 6.062789687331756E+6 - DDQ1 * cos(t28) * 3.1343757278E-5 + t14 * sin(t28) * 1.88062543668E-4 - 7.239948192E-3;
	tau[1] = DDQ2 * (-4.960294499772701E+7) + t3 * 4.047122001E-2 - t9 * 2.34465583186E-1 - t15 * 2.5192300599992E+9 + t57 * 1.73111503714E-1 - t58 * 4.047122001E-2 - t59 * 1.59282261872E-1 + t79 * 2.34465583186E-1 + DDQ2 * t3 * 3.329625500024844E+8 + DDQ2 * t9 * 1.471611370002782E+9 + DDQ2 * t57 * 4.374979600030794E+8 - DDQ2 * t58 * 5.537300750002691E+8 - DDQ2 * t59 * 5.38379718496E-4 + DDQ2 * t60 * 1.003000232896E-3 + DDQ2 * t63 * 1.168319682501076E+9 - DDQ2 * t79 * 1.578753700006539E+8 + t3 * t15 * 9.867798299986921E+8 + t9 * t15 * 2.85272283000271E+9 + t15 * t57 * 8.972708799992264E+9 + t15 * t58 * 4.681148800001481E+9 - t15 * t59 * 1.170519524991508E+9 - t15 * t60 * 9.134566299968447E+8 - t15 * t63 * 9.495164250367472E+7 - t15 * t79 * 3.254495680002862E+9 + DDQ2 * t9 * t58 * 2.15231007528E-3 + t9 * t15 * t58 * 1.984782880004E+9 + t9 * t15 * t59 * 1.940092699955234E+8 - t9 * t15 * t60 * 1.003000232896E-3;
	tau[2] = DDQ3 * 3.804646012528077E+8 + t4 * 9.4671504605E-3 - t10 * 1.87958303285E-2 - t16 * 9.711837499991606E+5 - t43 * 9.9876740941E-2 - t45 * 4.99383704705E-2 - t47 * 1.9910282734E-2 - t65 * 3.7591660657E-2 - t67 * 1.87958303285E-2 + DDQ3 * t4 * 3.067121287534485E+8 - DDQ3 * t10 * 2.901845644999087E+9 + DDQ3 * t43 * 3.501340800010343E+8 + DDQ3 * t45 * 1.956252462503944E+8 + DDQ3 * t47 * 1.1418207875E+8 - DDQ3 * t65 * 1.815647504999636E+9 - DDQ3 * t67 * 9.529240349998073E+8 - DDQ3 * t69 * 2.425115875012538E+7 - t4 * t16 * 2.666432333749714E+9 + t10 * t16 * 1.922112249836436E+7 - t16 * t43 * 3.512282937500688E+9 - t16 * t45 * 2.940115021874471E+9 - t16 * t47 * 3.367869937506888E+8 - t16 * t65 * 3.176799225004483E+8 - t16 * t67 * 2.031860381260668E+8 - t16 * t69 * 3.03078112500012E+8 - t16 * cos(t31) * 5.709103937486548E+7 + t16 * sin(t31) * 1.212557937509634E+7 - t16 * sin(Q3 * 6.0) * 3.1343757278E-5 - 3.9495238197E-2;
	tau[3] = DDQ4 * 6.625969125008182E+8 + t5 * 3.9820565468E-2 - t11 * 3.9820565468E-2 - t17 * 6.42195977258E-4 - t48 * 3.9820565468E-2 - t49 * 1.9910282734E-2 - t68 * 1.5414099498E-2 - t70 * 3.9820565468E-2 + DDQ4 * t5 * 1.435686036250608E+9 + DDQ4 * t11 * 7.862047624862266E+7 + DDQ4 * t46 * 1.832852872500759E+9 + DDQ4 * t48 * 1.138720214374856E+9 + DDQ4 * t49 * 3.8201436E+8 + DDQ4 * t50 * 1.2125579375E+7 + DDQ4 * t68 * 1.084301925006442E+8 + DDQ4 * t70 * 1.470269906251162E+8 + DDQ4 * t71 * 1.9128004125E+8 + DDQ4 * t72 * 5.7091039375E+7 + t5 * t17 * 7.862047624627257E+7 - t11 * t17 * 1.435686036249425E+9 + t17 * t46 * 2.168603849995674E+8 + t17 * t48 * 4.410809718750884E+8 + t17 * t49 * 7.651201649999144E+8 + t17 * t50 * 2.854551968751613E+8 - t17 * t68 * 3.665705744998855E+9 - t17 * t70 * 3.416160643123618E+9 - t17 * t71 * 1.528057439999892E+9 - t17 * t72 * 6.062789687488914E+7 - t17 * sin(Q4 * 6.0) * 3.1343757278E-5 + 1.9910282734E-2;
	tau[4] = DDQ5v * (-2.340216787500689E+8) + t13 * 2.401792785E-4 + t19 * 2.77799115769875E-4 + t76 * 8.187408306E-3 + t77 * 2.401792785E-4 - DDQ5v * t7 * 4.967856825002377E+8 - DDQ5v * t13 * 3.664701250007112E+7 - DDQ5v * t54 * 6.109509574999383E+8 - DDQ5v * t55 * 3.653896775000018E+8 - DDQ5v * t56 * 1.1418207875E+8 - DDQ5v * t76 * 1.237129249997896E+7 + DDQ5v * t77 * 2.424900249997096E+7 + DDQ5v * t78 * 2.425115875E+7 - t7 * t19 * 3.664701249967932E+7 + t13 * t19 * 4.967856825005336E+8 - t19 * t54 * 2.474258499988307E+7 + t19 * t55 * 7.274700749986551E+7 + t19 * t56 * 9.700463499998672E+7 + t19 * t76 * 1.221901914999877E+9 + t19 * t77 * 1.096169032499953E+9 + t19 * t78 * 4.567283149999982E+8 - t19 * cos(Q5v * 5.0) * 4.524762375E-7;
	tau[5] = DDQ5h * (-2.340216787500472E+8) - t12 * 2.401792785E-4 + t18 * 2.46711552436125E-4 + t73 * 7.226691192E-3 - t74 * 2.401792785E-4 - DDQ5h * t6 * 4.967856825002269E+8 - DDQ5h * t12 * 3.664701250007112E+7 - DDQ5h * t51 * 6.109509574999455E+8 - DDQ5h * t52 * 3.653896774999982E+8 - DDQ5h * t53 * 1.1418207875E+8 - DDQ5h * t73 * 1.237129249997896E+7 + DDQ5h * t74 * 2.424900249997096E+7 + DDQ5h * t75 * 2.425115875E+7 - t6 * t18 * 3.664701249975646E+7 + t12 * t18 * 4.967856825005118E+8 - t18 * t51 * 2.474258499991662E+7 + t18 * t52 * 7.274700749986543E+7 + t18 * t53 * 9.700463499998425E+7 + t18 * t73 * 1.221901914999891E+9 + t18 * t74 * 1.096169032499931E+9 + t18 * t75 * 4.567283150000018E+8 + t18 * cos(Q5h * 5.0) * 4.524762375E-7;
	for (int i = 0; i < 6; i++) {
		std::cout << tau[i] << std::endl;
	}
}


// application reads from the specified serial port and reports the collected data
int _tmain(int argc, _TCHAR* argv[])
{
	/*
	printf("Welcome to the serial test app!\n\n");

	arduinoCOM Arduino;

	

	if (Arduino.isConnected())
		printf("We're connected");



	while (!Arduino.isConnected())
	{

	Sleep(2000);
	//Test.setPosition(Test.getPosition(2, Test.Data) + 100, Test.Data, 2);
	}*/


	DynamicsTest(0,0,0,0,0,0);
	

	return 0;
}


