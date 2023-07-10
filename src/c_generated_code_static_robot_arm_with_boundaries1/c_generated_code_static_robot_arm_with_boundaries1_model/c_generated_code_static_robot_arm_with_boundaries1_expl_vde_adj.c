/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) c_generated_code_static_robot_arm_with_boundaries1_expl_vde_adj_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_sq CASADI_PREFIX(sq)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

casadi_real casadi_sq(casadi_real x) { return x*x;}

static const casadi_int casadi_s0[23] = {19, 1, 0, 19, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18};
static const casadi_int casadi_s1[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[24] = {20, 1, 0, 20, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};

/* c_generated_code_static_robot_arm_with_boundaries1_expl_vde_adj:(i0[19],i1[19],i2,i3[])->(o0[20]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a100, a101, a102, a103, a104, a105, a106, a107, a108, a109, a11, a110, a111, a112, a113, a114, a115, a116, a117, a118, a119, a12, a120, a121, a122, a123, a124, a125, a126, a127, a128, a129, a13, a130, a131, a132, a133, a134, a135, a136, a137, a138, a139, a14, a140, a141, a142, a143, a144, a145, a146, a147, a148, a149, a15, a150, a151, a152, a153, a154, a155, a156, a157, a158, a159, a16, a160, a161, a162, a163, a164, a165, a166, a167, a168, a169, a17, a170, a171, a172, a173, a174, a175, a176, a177, a178, a179, a18, a180, a181, a182, a183, a184, a185, a186, a187, a188, a189, a19, a190, a191, a192, a193, a194, a195, a196, a197, a198, a199, a2, a20, a200, a201, a202, a203, a204, a205, a206, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a6, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a7, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a8, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a9, a90, a91, a92, a93, a94, a95, a96, a97, a98, a99;
  a0=0.;
  if (res[0]!=0) res[0][0]=a0;
  if (res[0]!=0) res[0][1]=a0;
  if (res[0]!=0) res[0][2]=a0;
  a1=5.0929581789406498e+01;
  a2=2.;
  a3=arg[0]? arg[0][4] : 0;
  a4=arg[0]? arg[0][6] : 0;
  a5=(a3*a4);
  a6=arg[0]? arg[0][3] : 0;
  a7=arg[0]? arg[0][5] : 0;
  a8=(a6*a7);
  a5=(a5+a8);
  a5=(a2*a5);
  a8=(a1*a5);
  a9=arg[0]? arg[0][10] : 0;
  a10=(a8*a9);
  a11=(a7*a4);
  a12=(a6*a3);
  a11=(a11-a12);
  a11=(a2*a11);
  a12=(a1*a11);
  a13=arg[0]? arg[0][11] : 0;
  a14=(a12*a13);
  a10=(a10+a14);
  a14=casadi_sq(a6);
  a15=casadi_sq(a4);
  a14=(a14+a15);
  a14=(a2*a14);
  a15=1.;
  a14=(a14-a15);
  a16=(a1*a14);
  a17=arg[0]? arg[0][12] : 0;
  a18=(a16*a17);
  a10=(a10+a18);
  a18=5.0000000000000000e-01;
  a19=arg[1]? arg[1][6] : 0;
  a20=(a18*a19);
  a21=(a10*a20);
  a22=2.0371832715762599e+02;
  a23=(a3*a7);
  a24=(a6*a4);
  a23=(a23-a24);
  a23=(a2*a23);
  a24=(a22*a23);
  a25=(a24*a9);
  a26=casadi_sq(a6);
  a27=casadi_sq(a7);
  a26=(a26+a27);
  a26=(a2*a26);
  a26=(a26-a15);
  a27=(a22*a26);
  a28=(a27*a13);
  a25=(a25+a28);
  a28=(a7*a4);
  a29=(a6*a3);
  a28=(a28+a29);
  a28=(a2*a28);
  a29=(a22*a28);
  a30=(a29*a17);
  a25=(a25+a30);
  a30=arg[1]? arg[1][5] : 0;
  a31=(a18*a30);
  a32=(a25*a31);
  a21=(a21+a32);
  a32=casadi_sq(a6);
  a33=casadi_sq(a3);
  a32=(a32+a33);
  a32=(a2*a32);
  a32=(a32-a15);
  a33=(a22*a32);
  a34=(a33*a9);
  a35=(a3*a7);
  a36=(a6*a4);
  a35=(a35+a36);
  a35=(a2*a35);
  a36=(a22*a35);
  a37=(a36*a13);
  a34=(a34+a37);
  a37=(a3*a4);
  a38=(a6*a7);
  a37=(a37-a38);
  a37=(a2*a37);
  a38=(a22*a37);
  a39=(a38*a17);
  a34=(a34+a39);
  a39=arg[1]? arg[1][4] : 0;
  a40=(a18*a39);
  a41=(a34*a40);
  a21=(a21+a41);
  a41=1.0000000000000000e-02;
  a42=casadi_sq(a6);
  a43=casadi_sq(a3);
  a42=(a42+a43);
  a43=casadi_sq(a7);
  a42=(a42+a43);
  a43=casadi_sq(a4);
  a42=(a42+a43);
  a42=(a15-a42);
  a42=(a41*a42);
  a43=arg[1]? arg[1][3] : 0;
  a44=(a42*a43);
  a21=(a21+a44);
  a44=(a6+a6);
  a45=(a4*a19);
  a46=(a7*a30);
  a45=(a45+a46);
  a46=(a3*a39);
  a45=(a45+a46);
  a46=(a6*a43);
  a45=(a45+a46);
  a41=(a41*a45);
  a44=(a44*a41);
  a21=(a21-a44);
  a44=(a6+a6);
  a45=-2.1217559999999996e-02;
  a46=6.3661977236758142e-06;
  a47=(a46*a32);
  a48=arg[0]? arg[0][7] : 0;
  a49=(a47*a48);
  a50=(a46*a35);
  a51=arg[0]? arg[0][8] : 0;
  a52=(a50*a51);
  a49=(a49+a52);
  a52=(a46*a37);
  a53=arg[0]? arg[0][9] : 0;
  a54=(a52*a53);
  a49=(a49+a54);
  a54=(a37*a49);
  a55=(a46*a23);
  a56=(a55*a48);
  a57=(a46*a26);
  a58=(a57*a51);
  a56=(a56+a58);
  a58=(a46*a28);
  a59=(a58*a53);
  a56=(a56+a59);
  a59=(a28*a56);
  a54=(a54+a59);
  a59=1.2732395447351628e-05;
  a60=(a59*a5);
  a61=(a60*a48);
  a62=(a59*a11);
  a63=(a62*a51);
  a61=(a61+a63);
  a63=(a59*a14);
  a64=(a63*a53);
  a61=(a61+a64);
  a61=(a61+a15);
  a15=(a14*a61);
  a54=(a54+a15);
  a15=-1.2250000000000000e-02;
  a64=(a28*a10);
  a65=(a14*a25);
  a64=(a64-a65);
  a64=(a15*a64);
  a65=(a14*a34);
  a66=(a37*a10);
  a65=(a65-a66);
  a65=(a45*a65);
  a64=(a64+a65);
  a64=(a54+a64);
  a65=(a64+a64);
  a66=arg[1]? arg[1][18] : 0;
  a67=(a32*a49);
  a68=(a23*a56);
  a67=(a67+a68);
  a68=(a5*a61);
  a67=(a67+a68);
  a68=(a23*a10);
  a69=(a5*a25);
  a68=(a68-a69);
  a68=(a15*a68);
  a69=(a5*a34);
  a70=(a32*a10);
  a69=(a69-a70);
  a69=(a45*a69);
  a68=(a68+a69);
  a68=(a67+a68);
  a69=casadi_sq(a68);
  a70=(a35*a49);
  a71=(a26*a56);
  a70=(a70+a71);
  a71=(a11*a61);
  a70=(a70+a71);
  a71=(a26*a10);
  a72=(a11*a25);
  a71=(a71-a72);
  a71=(a15*a71);
  a72=(a11*a34);
  a73=(a35*a10);
  a72=(a72-a73);
  a72=(a45*a72);
  a71=(a71+a72);
  a71=(a70+a71);
  a72=casadi_sq(a71);
  a69=(a69+a72);
  a64=casadi_sq(a64);
  a69=(a69+a64);
  a69=sqrt(a69);
  a69=(a69+a69);
  a66=(a66/a69);
  a65=(a65*a66);
  a69=(a45*a65);
  a64=(a34*a69);
  a72=(a15*a65);
  a73=(a25*a72);
  a64=(a64-a73);
  a73=2.1217559999999996e-02;
  a74=(a28*a10);
  a75=(a14*a25);
  a74=(a74-a75);
  a74=(a15*a74);
  a75=(a14*a34);
  a76=(a37*a10);
  a75=(a75-a76);
  a75=(a73*a75);
  a74=(a74+a75);
  a74=(a54+a74);
  a75=(a74+a74);
  a76=arg[1]? arg[1][17] : 0;
  a77=(a23*a10);
  a78=(a5*a25);
  a77=(a77-a78);
  a77=(a15*a77);
  a78=(a5*a34);
  a79=(a32*a10);
  a78=(a78-a79);
  a78=(a73*a78);
  a77=(a77+a78);
  a77=(a67+a77);
  a78=casadi_sq(a77);
  a79=(a26*a10);
  a80=(a11*a25);
  a79=(a79-a80);
  a79=(a15*a79);
  a80=(a11*a34);
  a81=(a35*a10);
  a80=(a80-a81);
  a80=(a73*a80);
  a79=(a79+a80);
  a79=(a70+a79);
  a80=casadi_sq(a79);
  a78=(a78+a80);
  a74=casadi_sq(a74);
  a78=(a78+a74);
  a78=sqrt(a78);
  a78=(a78+a78);
  a76=(a76/a78);
  a75=(a75*a76);
  a78=(a73*a75);
  a74=(a34*a78);
  a64=(a64+a74);
  a74=(a15*a75);
  a80=(a25*a74);
  a64=(a64-a80);
  a80=2.7999999999999997e-02;
  a81=(a28*a10);
  a82=(a14*a25);
  a81=(a81-a82);
  a81=(a80*a81);
  a81=(a54+a81);
  a82=(a81+a81);
  a83=arg[1]? arg[1][16] : 0;
  a84=(a23*a10);
  a85=(a5*a25);
  a84=(a84-a85);
  a84=(a80*a84);
  a84=(a67+a84);
  a85=casadi_sq(a84);
  a86=(a26*a10);
  a87=(a11*a25);
  a86=(a86-a87);
  a86=(a80*a86);
  a86=(a70+a86);
  a87=casadi_sq(a86);
  a85=(a85+a87);
  a81=casadi_sq(a81);
  a85=(a85+a81);
  a85=sqrt(a85);
  a85=(a85+a85);
  a83=(a83/a85);
  a82=(a82*a83);
  a85=(a80*a82);
  a81=(a25*a85);
  a64=(a64-a81);
  a81=arg[0]? arg[0][15] : 0;
  a87=(a37*a49);
  a88=(a28*a56);
  a87=(a87+a88);
  a88=(a14*a61);
  a87=(a87+a88);
  a88=(a35*a49);
  a89=(a26*a56);
  a88=(a88+a89);
  a89=(a11*a61);
  a88=(a88+a89);
  a89=(a87*a88);
  a90=(a81*a89);
  a91=arg[1]? arg[1][8] : 0;
  a92=(a32*a49);
  a93=(a23*a56);
  a92=(a92+a93);
  a93=(a5*a61);
  a92=(a92+a93);
  a93=casadi_sq(a92);
  a94=casadi_sq(a88);
  a93=(a93+a94);
  a94=casadi_sq(a87);
  a93=(a93+a94);
  a94=sqrt(a93);
  a95=(a94*a93);
  a96=(a91/a95);
  a97=(a90*a96);
  a98=casadi_sq(a88);
  a99=casadi_sq(a92);
  a98=(a98+a99);
  a99=(a81*a98);
  a100=arg[1]? arg[1][9] : 0;
  a101=(a100/a95);
  a102=(a99*a101);
  a103=arg[0]? arg[0][14] : 0;
  a104=casadi_sq(a88);
  a105=casadi_sq(a92);
  a104=(a104+a105);
  a105=(a103*a104);
  a106=casadi_sq(a92);
  a107=casadi_sq(a88);
  a106=(a106+a107);
  a107=casadi_sq(a87);
  a106=(a106+a107);
  a107=sqrt(a106);
  a108=(a107*a106);
  a109=(a100/a108);
  a110=(a105*a109);
  a102=(a102+a110);
  a110=arg[0]? arg[0][13] : 0;
  a111=casadi_sq(a88);
  a112=casadi_sq(a92);
  a111=(a111+a112);
  a112=(a110*a111);
  a113=casadi_sq(a92);
  a114=casadi_sq(a88);
  a113=(a113+a114);
  a114=casadi_sq(a87);
  a113=(a113+a114);
  a114=sqrt(a113);
  a115=(a114*a113);
  a116=(a100/a115);
  a117=(a112*a116);
  a102=(a102+a117);
  a97=(a97-a102);
  a102=(a87*a88);
  a117=(a103*a102);
  a118=(a91/a108);
  a119=(a117*a118);
  a97=(a97+a119);
  a119=(a87*a88);
  a120=(a110*a119);
  a121=(a91/a115);
  a122=(a120*a121);
  a97=(a97+a122);
  a122=(a87*a92);
  a123=(a81*a122);
  a124=arg[1]? arg[1][7] : 0;
  a125=(a124/a95);
  a126=(a123*a125);
  a97=(a97+a126);
  a126=(a87*a92);
  a127=(a103*a126);
  a128=(a124/a108);
  a129=(a127*a128);
  a97=(a97+a129);
  a129=(a87*a92);
  a130=(a110*a129);
  a131=(a124/a115);
  a132=(a130*a131);
  a97=(a97+a132);
  a132=(a56*a97);
  a133=(a34*a132);
  a64=(a64+a133);
  a133=(a49*a97);
  a134=(a25*a133);
  a64=(a64-a134);
  a134=(a26*a10);
  a135=(a11*a25);
  a134=(a134-a135);
  a135=(a134*a49);
  a136=(a11*a34);
  a137=(a35*a10);
  a136=(a136-a137);
  a137=(a136*a56);
  a135=(a135+a137);
  a137=(a35*a25);
  a138=(a26*a34);
  a137=(a137-a138);
  a138=(a137*a61);
  a135=(a135+a138);
  a138=(a135*a101);
  a139=(a81*a138);
  a140=(a88*a139);
  a141=(a23*a10);
  a142=(a5*a25);
  a141=(a141-a142);
  a142=(a141*a49);
  a143=(a5*a34);
  a144=(a32*a10);
  a143=(a143-a144);
  a144=(a143*a56);
  a142=(a142+a144);
  a144=(a32*a25);
  a145=(a23*a34);
  a144=(a144-a145);
  a145=(a144*a61);
  a142=(a142+a145);
  a145=(a142*a101);
  a146=(a81*a145);
  a147=(a92*a146);
  a140=(a140+a147);
  a147=(a135*a109);
  a148=(a103*a147);
  a149=(a88*a148);
  a140=(a140+a149);
  a149=(a142*a109);
  a150=(a103*a149);
  a151=(a92*a150);
  a140=(a140+a151);
  a151=(a135*a116);
  a152=(a110*a151);
  a153=(a88*a152);
  a140=(a140+a153);
  a153=(a142*a116);
  a154=(a110*a153);
  a155=(a92*a154);
  a140=(a140+a155);
  a155=(a28*a10);
  a156=(a14*a25);
  a155=(a155-a156);
  a156=(a155*a49);
  a157=(a14*a34);
  a158=(a37*a10);
  a157=(a157-a158);
  a158=(a157*a56);
  a156=(a156+a158);
  a158=(a37*a25);
  a159=(a28*a34);
  a158=(a158-a159);
  a159=(a158*a61);
  a156=(a156+a159);
  a159=(a156*a96);
  a160=(a81*a159);
  a161=(a88*a160);
  a140=(a140+a161);
  a161=(a87+a87);
  a162=(a135*a96);
  a163=(a81*a162);
  a161=(a161*a163);
  a140=(a140-a161);
  a161=(a156*a118);
  a164=(a103*a161);
  a165=(a88*a164);
  a140=(a140+a165);
  a165=(a87+a87);
  a166=(a135*a118);
  a167=(a103*a166);
  a165=(a165*a167);
  a140=(a140-a165);
  a165=(a156*a121);
  a168=(a110*a165);
  a169=(a88*a168);
  a140=(a140+a169);
  a169=(a87+a87);
  a170=(a135*a121);
  a171=(a110*a170);
  a169=(a169*a171);
  a140=(a140-a169);
  a169=(a87+a87);
  a172=(a92*a87);
  a173=(a81*a172);
  a174=(a173*a142);
  a175=(a88*a87);
  a176=(a81*a175);
  a177=(a176*a135);
  a174=(a174+a177);
  a99=(a99*a156);
  a174=(a174-a99);
  a174=(a174/a95);
  a174=(a174/a95);
  a174=(a174*a100);
  a99=(a92*a88);
  a177=(a81*a99);
  a178=(a177*a142);
  a179=casadi_sq(a87);
  a180=casadi_sq(a92);
  a179=(a179+a180);
  a180=(a81*a179);
  a181=(a180*a135);
  a178=(a178-a181);
  a90=(a90*a156);
  a178=(a178+a90);
  a178=(a178/a95);
  a178=(a178/a95);
  a178=(a178*a91);
  a174=(a174+a178);
  a178=(a88*a92);
  a90=(a81*a178);
  a181=(a90*a135);
  a182=casadi_sq(a87);
  a183=casadi_sq(a88);
  a182=(a182+a183);
  a183=(a81*a182);
  a184=(a183*a142);
  a181=(a181-a184);
  a123=(a123*a156);
  a181=(a181+a123);
  a181=(a181/a95);
  a181=(a181/a95);
  a181=(a181*a124);
  a174=(a174+a181);
  a181=(a94*a174);
  a93=(a93*a174);
  a94=(a94+a94);
  a93=(a93/a94);
  a181=(a181+a93);
  a169=(a169*a181);
  a140=(a140-a169);
  a169=(a156*a125);
  a93=(a81*a169);
  a94=(a92*a93);
  a140=(a140+a94);
  a94=(a87+a87);
  a174=(a142*a125);
  a95=(a81*a174);
  a94=(a94*a95);
  a140=(a140-a94);
  a94=(a87+a87);
  a123=(a92*a87);
  a184=(a103*a123);
  a185=(a184*a142);
  a186=(a88*a87);
  a187=(a103*a186);
  a188=(a187*a135);
  a185=(a185+a188);
  a105=(a105*a156);
  a185=(a185-a105);
  a185=(a185/a108);
  a185=(a185/a108);
  a185=(a185*a100);
  a105=(a92*a88);
  a188=(a103*a105);
  a189=(a188*a142);
  a190=casadi_sq(a87);
  a191=casadi_sq(a92);
  a190=(a190+a191);
  a191=(a103*a190);
  a192=(a191*a135);
  a189=(a189-a192);
  a117=(a117*a156);
  a189=(a189+a117);
  a189=(a189/a108);
  a189=(a189/a108);
  a189=(a189*a91);
  a185=(a185+a189);
  a189=(a88*a92);
  a117=(a103*a189);
  a192=(a117*a135);
  a193=casadi_sq(a87);
  a194=casadi_sq(a88);
  a193=(a193+a194);
  a194=(a103*a193);
  a195=(a194*a142);
  a192=(a192-a195);
  a127=(a127*a156);
  a192=(a192+a127);
  a192=(a192/a108);
  a192=(a192/a108);
  a192=(a192*a124);
  a185=(a185+a192);
  a192=(a107*a185);
  a106=(a106*a185);
  a107=(a107+a107);
  a106=(a106/a107);
  a192=(a192+a106);
  a94=(a94*a192);
  a140=(a140-a94);
  a94=(a156*a128);
  a106=(a103*a94);
  a107=(a92*a106);
  a140=(a140+a107);
  a107=(a87+a87);
  a185=(a142*a128);
  a108=(a103*a185);
  a107=(a107*a108);
  a140=(a140-a107);
  a107=(a87+a87);
  a127=(a92*a87);
  a195=(a110*a127);
  a196=(a195*a142);
  a197=(a88*a87);
  a198=(a110*a197);
  a199=(a198*a135);
  a196=(a196+a199);
  a112=(a112*a156);
  a196=(a196-a112);
  a196=(a196/a115);
  a196=(a196/a115);
  a196=(a196*a100);
  a100=(a92*a88);
  a112=(a110*a100);
  a199=(a112*a142);
  a200=casadi_sq(a87);
  a201=casadi_sq(a92);
  a200=(a200+a201);
  a201=(a110*a200);
  a202=(a201*a135);
  a199=(a199-a202);
  a120=(a120*a156);
  a199=(a199+a120);
  a199=(a199/a115);
  a199=(a199/a115);
  a199=(a199*a91);
  a196=(a196+a199);
  a199=(a88*a92);
  a91=(a110*a199);
  a120=(a91*a135);
  a202=casadi_sq(a87);
  a203=casadi_sq(a88);
  a202=(a202+a203);
  a203=(a110*a202);
  a204=(a203*a142);
  a120=(a120-a204);
  a130=(a130*a156);
  a120=(a120+a130);
  a120=(a120/a115);
  a120=(a120/a115);
  a120=(a120*a124);
  a196=(a196+a120);
  a120=(a114*a196);
  a113=(a113*a196);
  a114=(a114+a114);
  a113=(a113/a114);
  a120=(a120+a113);
  a107=(a107*a120);
  a140=(a140-a107);
  a107=(a156*a131);
  a113=(a110*a107);
  a114=(a92*a113);
  a140=(a140+a114);
  a114=(a87+a87);
  a196=(a142*a131);
  a124=(a110*a196);
  a114=(a114*a124);
  a140=(a140-a114);
  a114=(a61*a140);
  a64=(a64+a114);
  a114=(a28*a72);
  a115=(a37*a69);
  a114=(a114-a115);
  a71=(a71+a71);
  a71=(a71*a66);
  a115=(a45*a71);
  a130=(a35*a115);
  a114=(a114-a130);
  a130=(a15*a71);
  a204=(a26*a130);
  a114=(a114+a204);
  a68=(a68+a68);
  a68=(a68*a66);
  a45=(a45*a68);
  a66=(a32*a45);
  a114=(a114-a66);
  a66=(a15*a68);
  a204=(a23*a66);
  a114=(a114+a204);
  a204=(a37*a78);
  a114=(a114-a204);
  a204=(a28*a74);
  a114=(a114+a204);
  a79=(a79+a79);
  a79=(a79*a76);
  a204=(a73*a79);
  a205=(a35*a204);
  a114=(a114-a205);
  a205=(a15*a79);
  a206=(a26*a205);
  a114=(a114+a206);
  a77=(a77+a77);
  a77=(a77*a76);
  a73=(a73*a77);
  a76=(a32*a73);
  a114=(a114-a76);
  a15=(a15*a77);
  a76=(a23*a15);
  a114=(a114+a76);
  a76=(a28*a85);
  a114=(a114+a76);
  a86=(a86+a86);
  a86=(a86*a83);
  a76=(a80*a86);
  a206=(a26*a76);
  a114=(a114+a206);
  a84=(a84+a84);
  a84=(a84*a83);
  a80=(a80*a84);
  a83=(a23*a80);
  a114=(a114+a83);
  a83=(a37*a132);
  a114=(a114-a83);
  a83=(a28*a133);
  a114=(a114+a83);
  a173=(a173*a101);
  a184=(a184*a109);
  a173=(a173+a184);
  a195=(a195*a116);
  a173=(a173+a195);
  a177=(a177*a96);
  a173=(a173+a177);
  a188=(a188*a118);
  a173=(a173+a188);
  a112=(a112*a121);
  a173=(a173+a112);
  a183=(a183*a125);
  a173=(a173-a183);
  a194=(a194*a128);
  a173=(a173-a194);
  a203=(a203*a131);
  a173=(a173-a203);
  a203=(a56*a173);
  a194=(a32*a203);
  a114=(a114-a194);
  a194=(a49*a173);
  a183=(a23*a194);
  a114=(a114+a183);
  a176=(a176*a101);
  a187=(a187*a109);
  a176=(a176+a187);
  a198=(a198*a116);
  a176=(a176+a198);
  a180=(a180*a96);
  a176=(a176-a180);
  a191=(a191*a118);
  a176=(a176-a191);
  a201=(a201*a121);
  a176=(a176-a201);
  a90=(a90*a125);
  a176=(a176+a90);
  a117=(a117*a128);
  a176=(a176+a117);
  a91=(a91*a131);
  a176=(a176+a91);
  a91=(a56*a176);
  a117=(a35*a91);
  a114=(a114-a117);
  a117=(a49*a176);
  a90=(a26*a117);
  a114=(a114+a90);
  a90=(a6*a20);
  a114=(a114+a90);
  a90=(a3*a31);
  a114=(a114-a90);
  a90=(a7*a40);
  a114=(a114+a90);
  a18=(a18*a43);
  a43=(a4*a18);
  a114=(a114-a43);
  a43=(a17*a114);
  a43=(a1*a43);
  a64=(a64+a43);
  a65=(a65+a75);
  a65=(a65+a82);
  a82=arg[1]? arg[1][11] : 0;
  a75=(a48*a82);
  a65=(a65-a75);
  a75=arg[1]? arg[1][10] : 0;
  a43=(a51*a75);
  a65=(a65+a43);
  a43=arg[1]? arg[1][2] : 0;
  a65=(a65+a43);
  a43=(a61*a65);
  a64=(a64+a43);
  a158=(a158*a97);
  a144=(a144*a173);
  a158=(a158+a144);
  a144=(a14*a140);
  a158=(a158+a144);
  a137=(a137*a176);
  a158=(a158+a137);
  a146=(a87*a146);
  a137=(a92+a92);
  a101=(a156*a101);
  a144=(a81*a101);
  a137=(a137*a144);
  a146=(a146-a137);
  a137=(a92+a92);
  a109=(a156*a109);
  a43=(a103*a109);
  a137=(a137*a43);
  a146=(a146-a137);
  a150=(a87*a150);
  a146=(a146+a150);
  a150=(a92+a92);
  a156=(a156*a116);
  a116=(a110*a156);
  a150=(a150*a116);
  a146=(a146-a150);
  a154=(a87*a154);
  a146=(a146+a154);
  a154=(a92+a92);
  a154=(a154*a163);
  a146=(a146-a154);
  a96=(a142*a96);
  a154=(a81*a96);
  a163=(a88*a154);
  a146=(a146+a163);
  a163=(a92+a92);
  a163=(a163*a167);
  a146=(a146-a163);
  a118=(a142*a118);
  a163=(a103*a118);
  a167=(a88*a163);
  a146=(a146+a167);
  a167=(a92+a92);
  a167=(a167*a171);
  a146=(a146-a167);
  a142=(a142*a121);
  a121=(a110*a142);
  a167=(a88*a121);
  a146=(a146+a167);
  a167=(a92+a92);
  a167=(a167*a181);
  a146=(a146-a167);
  a93=(a87*a93);
  a146=(a146+a93);
  a125=(a135*a125);
  a81=(a81*a125);
  a93=(a88*a81);
  a146=(a146+a93);
  a93=(a92+a92);
  a93=(a93*a192);
  a146=(a146-a93);
  a106=(a87*a106);
  a146=(a146+a106);
  a128=(a135*a128);
  a103=(a103*a128);
  a106=(a88*a103);
  a146=(a146+a106);
  a106=(a92+a92);
  a106=(a106*a120);
  a146=(a146-a106);
  a113=(a87*a113);
  a146=(a146+a113);
  a135=(a135*a131);
  a110=(a110*a135);
  a131=(a88*a110);
  a146=(a146+a131);
  a131=(a5*a146);
  a158=(a158+a131);
  a139=(a87*a139);
  a131=(a88+a88);
  a131=(a131*a144);
  a139=(a139-a131);
  a131=(a88+a88);
  a131=(a131*a43);
  a139=(a139-a131);
  a148=(a87*a148);
  a139=(a139+a148);
  a148=(a88+a88);
  a148=(a148*a116);
  a139=(a139-a148);
  a152=(a87*a152);
  a139=(a139+a152);
  a160=(a87*a160);
  a139=(a139+a160);
  a154=(a92*a154);
  a139=(a139+a154);
  a164=(a87*a164);
  a139=(a139+a164);
  a163=(a92*a163);
  a139=(a139+a163);
  a87=(a87*a168);
  a139=(a139+a87);
  a121=(a92*a121);
  a139=(a139+a121);
  a121=(a88+a88);
  a121=(a121*a181);
  a139=(a139-a121);
  a121=(a88+a88);
  a121=(a121*a95);
  a139=(a139-a121);
  a81=(a92*a81);
  a139=(a139+a81);
  a81=(a88+a88);
  a81=(a81*a192);
  a139=(a139-a81);
  a81=(a88+a88);
  a81=(a81*a108);
  a139=(a139-a81);
  a103=(a92*a103);
  a139=(a139+a103);
  a103=(a88+a88);
  a103=(a103*a120);
  a139=(a139-a103);
  a88=(a88+a88);
  a88=(a88*a124);
  a139=(a139-a88);
  a92=(a92*a110);
  a139=(a139+a92);
  a92=(a11*a139);
  a158=(a158+a92);
  a92=(a14*a65);
  a158=(a158+a92);
  a71=(a71+a79);
  a71=(a71+a86);
  a86=arg[1]? arg[1][12] : 0;
  a79=(a48*a86);
  a71=(a71+a79);
  a79=(a53*a75);
  a71=(a71-a79);
  a79=arg[1]? arg[1][1] : 0;
  a71=(a71+a79);
  a79=(a11*a71);
  a158=(a158+a79);
  a68=(a68+a77);
  a68=(a68+a84);
  a84=(a51*a86);
  a68=(a68-a84);
  a84=(a53*a82);
  a68=(a68+a84);
  a84=arg[1]? arg[1][0] : 0;
  a68=(a68+a84);
  a84=(a5*a68);
  a158=(a158+a84);
  a84=(a53*a158);
  a84=(a59*a84);
  a64=(a64+a84);
  a64=(a2*a64);
  a44=(a44*a64);
  a21=(a21+a44);
  a44=(a34*a115);
  a84=(a25*a130);
  a44=(a44-a84);
  a84=(a34*a204);
  a44=(a44+a84);
  a84=(a25*a205);
  a44=(a44-a84);
  a84=(a25*a76);
  a44=(a44-a84);
  a84=(a34*a91);
  a44=(a44+a84);
  a84=(a25*a117);
  a44=(a44-a84);
  a84=(a61*a139);
  a44=(a44+a84);
  a84=(a13*a114);
  a84=(a1*a84);
  a44=(a44+a84);
  a84=(a61*a71);
  a44=(a44+a84);
  a84=(a51*a158);
  a84=(a59*a84);
  a44=(a44+a84);
  a44=(a2*a44);
  a84=(a3*a44);
  a21=(a21-a84);
  a84=(a34*a45);
  a77=(a25*a66);
  a84=(a84-a77);
  a77=(a34*a73);
  a84=(a84+a77);
  a77=(a25*a15);
  a84=(a84-a77);
  a77=(a25*a80);
  a84=(a84-a77);
  a77=(a34*a203);
  a84=(a84+a77);
  a77=(a25*a194);
  a84=(a84-a77);
  a77=(a61*a146);
  a84=(a84+a77);
  a77=(a9*a114);
  a1=(a1*a77);
  a84=(a84+a1);
  a1=(a61*a68);
  a84=(a84+a1);
  a1=(a48*a158);
  a59=(a59*a1);
  a84=(a84+a59);
  a84=(a2*a84);
  a59=(a7*a84);
  a21=(a21+a59);
  a59=(a10*a72);
  a1=(a10*a74);
  a59=(a59+a1);
  a1=(a10*a85);
  a59=(a59+a1);
  a1=(a61*a97);
  a77=(a34*a1);
  a59=(a59-a77);
  a77=(a10*a133);
  a59=(a59+a77);
  a77=(a56*a140);
  a59=(a59+a77);
  a77=(a37*a1);
  a72=(a14*a72);
  a79=(a11*a130);
  a72=(a72+a79);
  a79=(a5*a66);
  a72=(a72+a79);
  a74=(a14*a74);
  a72=(a72+a74);
  a74=(a11*a205);
  a72=(a72+a74);
  a74=(a5*a15);
  a72=(a72+a74);
  a85=(a14*a85);
  a72=(a72+a85);
  a85=(a11*a76);
  a72=(a72+a85);
  a85=(a5*a80);
  a72=(a72+a85);
  a77=(a77-a72);
  a133=(a14*a133);
  a77=(a77-a133);
  a133=(a61*a173);
  a72=(a32*a133);
  a77=(a77+a72);
  a72=(a5*a194);
  a77=(a77-a72);
  a61=(a61*a176);
  a72=(a35*a61);
  a77=(a77+a72);
  a72=(a11*a117);
  a77=(a77-a72);
  a72=(a3*a20);
  a77=(a77+a72);
  a72=(a6*a31);
  a77=(a77+a72);
  a72=(a4*a40);
  a77=(a77-a72);
  a72=(a7*a18);
  a77=(a77-a72);
  a72=(a17*a77);
  a72=(a22*a72);
  a59=(a59+a72);
  a72=(a56*a65);
  a59=(a59+a72);
  a157=(a157*a97);
  a143=(a143*a173);
  a157=(a157+a143);
  a143=(a28*a140);
  a157=(a157+a143);
  a136=(a136*a176);
  a157=(a157+a136);
  a136=(a23*a146);
  a157=(a157+a136);
  a136=(a26*a139);
  a157=(a157+a136);
  a136=(a28*a65);
  a157=(a157+a136);
  a136=(a26*a71);
  a157=(a157+a136);
  a136=(a23*a68);
  a157=(a157+a136);
  a136=(a53*a157);
  a136=(a46*a136);
  a59=(a59+a136);
  a59=(a2*a59);
  a136=(a3*a59);
  a21=(a21+a136);
  a136=(a6+a6);
  a130=(a10*a130);
  a205=(a10*a205);
  a130=(a130+a205);
  a76=(a10*a76);
  a130=(a130+a76);
  a76=(a34*a61);
  a130=(a130-a76);
  a117=(a10*a117);
  a130=(a130+a117);
  a117=(a56*a139);
  a130=(a130+a117);
  a117=(a13*a77);
  a117=(a22*a117);
  a130=(a130+a117);
  a117=(a56*a71);
  a130=(a130+a117);
  a117=(a51*a157);
  a117=(a46*a117);
  a130=(a130+a117);
  a130=(a2*a130);
  a136=(a136*a130);
  a21=(a21+a136);
  a66=(a10*a66);
  a15=(a10*a15);
  a66=(a66+a15);
  a80=(a10*a80);
  a66=(a66+a80);
  a80=(a34*a133);
  a66=(a66-a80);
  a194=(a10*a194);
  a66=(a66+a194);
  a194=(a56*a146);
  a66=(a66+a194);
  a194=(a9*a77);
  a194=(a22*a194);
  a66=(a66+a194);
  a56=(a56*a68);
  a66=(a66+a56);
  a56=(a48*a157);
  a56=(a46*a56);
  a66=(a66+a56);
  a66=(a2*a66);
  a56=(a4*a66);
  a21=(a21-a56);
  a56=(a25*a1);
  a194=(a10*a69);
  a80=(a10*a78);
  a194=(a194+a80);
  a56=(a56-a194);
  a194=(a10*a132);
  a56=(a56-a194);
  a194=(a49*a140);
  a56=(a56+a194);
  a69=(a14*a69);
  a194=(a11*a115);
  a69=(a69+a194);
  a194=(a5*a45);
  a69=(a69+a194);
  a78=(a14*a78);
  a69=(a69+a78);
  a78=(a11*a204);
  a69=(a69+a78);
  a78=(a5*a73);
  a69=(a69+a78);
  a28=(a28*a1);
  a69=(a69-a28);
  a14=(a14*a132);
  a69=(a69+a14);
  a23=(a23*a133);
  a69=(a69-a23);
  a5=(a5*a203);
  a69=(a69+a5);
  a26=(a26*a61);
  a69=(a69-a26);
  a11=(a11*a91);
  a69=(a69+a11);
  a11=(a7*a20);
  a69=(a69-a11);
  a11=(a4*a31);
  a69=(a69+a11);
  a11=(a6*a40);
  a69=(a69+a11);
  a11=(a3*a18);
  a69=(a69-a11);
  a17=(a17*a69);
  a17=(a22*a17);
  a56=(a56+a17);
  a17=(a49*a65);
  a56=(a56+a17);
  a155=(a155*a97);
  a141=(a141*a173);
  a155=(a155+a141);
  a140=(a37*a140);
  a155=(a155+a140);
  a134=(a134*a176);
  a155=(a155+a134);
  a134=(a32*a146);
  a155=(a155+a134);
  a134=(a35*a139);
  a155=(a155+a134);
  a37=(a37*a65);
  a155=(a155+a37);
  a35=(a35*a71);
  a155=(a155+a35);
  a32=(a32*a68);
  a155=(a155+a32);
  a53=(a53*a155);
  a53=(a46*a53);
  a56=(a56+a53);
  a56=(a2*a56);
  a53=(a7*a56);
  a21=(a21-a53);
  a61=(a25*a61);
  a115=(a10*a115);
  a204=(a10*a204);
  a115=(a115+a204);
  a61=(a61-a115);
  a91=(a10*a91);
  a61=(a61-a91);
  a139=(a49*a139);
  a61=(a61+a139);
  a13=(a13*a69);
  a13=(a22*a13);
  a61=(a61+a13);
  a71=(a49*a71);
  a61=(a61+a71);
  a51=(a51*a155);
  a51=(a46*a51);
  a61=(a61+a51);
  a61=(a2*a61);
  a51=(a4*a61);
  a21=(a21+a51);
  a51=(a6+a6);
  a133=(a25*a133);
  a45=(a10*a45);
  a73=(a10*a73);
  a45=(a45+a73);
  a133=(a133-a45);
  a203=(a10*a203);
  a133=(a133-a203);
  a146=(a49*a146);
  a133=(a133+a146);
  a9=(a9*a69);
  a22=(a22*a9);
  a133=(a133+a22);
  a49=(a49*a68);
  a133=(a133+a49);
  a48=(a48*a155);
  a46=(a46*a48);
  a133=(a133+a46);
  a2=(a2*a133);
  a51=(a51*a2);
  a21=(a21+a51);
  if (res[0]!=0) res[0][3]=a21;
  a21=(a25*a20);
  a51=(a10*a31);
  a21=(a21-a51);
  a39=(a42*a39);
  a21=(a21+a39);
  a39=(a34*a18);
  a21=(a21-a39);
  a39=(a3+a3);
  a39=(a39*a41);
  a21=(a21-a39);
  a39=(a6*a44);
  a21=(a21-a39);
  a39=(a4*a84);
  a21=(a21+a39);
  a39=(a6*a59);
  a21=(a21+a39);
  a39=(a7*a66);
  a21=(a21+a39);
  a39=(a4*a56);
  a21=(a21+a39);
  a39=(a7*a61);
  a21=(a21+a39);
  a39=(a3+a3);
  a39=(a39*a2);
  a21=(a21+a39);
  if (res[0]!=0) res[0][4]=a21;
  a30=(a42*a30);
  a20=(a34*a20);
  a30=(a30-a20);
  a20=(a10*a40);
  a30=(a30+a20);
  a20=(a25*a18);
  a30=(a30-a20);
  a20=(a7+a7);
  a20=(a20*a41);
  a30=(a30-a20);
  a20=(a4*a44);
  a30=(a30+a20);
  a20=(a6*a84);
  a30=(a30+a20);
  a20=(a4*a59);
  a30=(a30+a20);
  a20=(a7+a7);
  a20=(a20*a130);
  a30=(a30+a20);
  a20=(a3*a66);
  a30=(a30+a20);
  a20=(a6*a56);
  a30=(a30-a20);
  a20=(a3*a61);
  a30=(a30+a20);
  if (res[0]!=0) res[0][5]=a30;
  a42=(a42*a19);
  a34=(a34*a31);
  a42=(a42+a34);
  a25=(a25*a40);
  a42=(a42-a25);
  a10=(a10*a18);
  a42=(a42-a10);
  a10=(a4+a4);
  a10=(a10*a41);
  a42=(a42-a10);
  a4=(a4+a4);
  a4=(a4*a64);
  a42=(a42+a4);
  a44=(a7*a44);
  a42=(a42+a44);
  a84=(a3*a84);
  a42=(a42+a84);
  a7=(a7*a59);
  a42=(a42+a7);
  a66=(a6*a66);
  a42=(a42-a66);
  a3=(a3*a56);
  a42=(a42+a3);
  a6=(a6*a61);
  a42=(a42+a6);
  if (res[0]!=0) res[0][6]=a42;
  a42=(a70*a86);
  a6=(a54*a82);
  a42=(a42-a6);
  a60=(a60*a158);
  a42=(a42+a60);
  a55=(a55*a157);
  a42=(a42+a55);
  a47=(a47*a155);
  a42=(a42+a47);
  if (res[0]!=0) res[0][7]=a42;
  a54=(a54*a75);
  a86=(a67*a86);
  a54=(a54-a86);
  a62=(a62*a158);
  a54=(a54+a62);
  a57=(a57*a157);
  a54=(a54+a57);
  a50=(a50*a155);
  a54=(a54+a50);
  if (res[0]!=0) res[0][8]=a54;
  a67=(a67*a82);
  a70=(a70*a75);
  a67=(a67-a70);
  a63=(a63*a158);
  a67=(a67+a63);
  a58=(a58*a157);
  a67=(a67+a58);
  a52=(a52*a155);
  a67=(a67+a52);
  if (res[0]!=0) res[0][9]=a67;
  a8=(a8*a114);
  a24=(a24*a77);
  a8=(a8+a24);
  a33=(a33*a69);
  a8=(a8+a33);
  if (res[0]!=0) res[0][10]=a8;
  a12=(a12*a114);
  a27=(a27*a77);
  a12=(a12+a27);
  a36=(a36*a69);
  a12=(a12+a36);
  if (res[0]!=0) res[0][11]=a12;
  a16=(a16*a114);
  a29=(a29*a77);
  a16=(a16+a29);
  a38=(a38*a69);
  a16=(a16+a38);
  if (res[0]!=0) res[0][12]=a16;
  a197=(a197*a151);
  a111=(a111*a156);
  a197=(a197-a111);
  a127=(a127*a153);
  a197=(a197+a127);
  a119=(a119*a165);
  a197=(a197+a119);
  a200=(a200*a170);
  a197=(a197-a200);
  a100=(a100*a142);
  a197=(a197+a100);
  a129=(a129*a107);
  a197=(a197+a129);
  a202=(a202*a196);
  a197=(a197-a202);
  a199=(a199*a135);
  a197=(a197+a199);
  if (res[0]!=0) res[0][13]=a197;
  a186=(a186*a147);
  a104=(a104*a109);
  a186=(a186-a104);
  a123=(a123*a149);
  a186=(a186+a123);
  a102=(a102*a161);
  a186=(a186+a102);
  a190=(a190*a166);
  a186=(a186-a190);
  a105=(a105*a118);
  a186=(a186+a105);
  a126=(a126*a94);
  a186=(a186+a126);
  a193=(a193*a185);
  a186=(a186-a193);
  a189=(a189*a128);
  a186=(a186+a189);
  if (res[0]!=0) res[0][14]=a186;
  a175=(a175*a138);
  a98=(a98*a101);
  a175=(a175-a98);
  a172=(a172*a145);
  a175=(a175+a172);
  a89=(a89*a159);
  a175=(a175+a89);
  a179=(a179*a162);
  a175=(a175-a179);
  a99=(a99*a96);
  a175=(a175+a99);
  a122=(a122*a169);
  a175=(a175+a122);
  a182=(a182*a174);
  a175=(a175-a182);
  a178=(a178*a125);
  a175=(a175+a178);
  if (res[0]!=0) res[0][15]=a175;
  if (res[0]!=0) res[0][16]=a0;
  if (res[0]!=0) res[0][17]=a0;
  if (res[0]!=0) res[0][18]=a0;
  if (res[0]!=0) res[0][19]=a0;
  return 0;
}

CASADI_SYMBOL_EXPORT int c_generated_code_static_robot_arm_with_boundaries1_expl_vde_adj(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int c_generated_code_static_robot_arm_with_boundaries1_expl_vde_adj_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int c_generated_code_static_robot_arm_with_boundaries1_expl_vde_adj_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void c_generated_code_static_robot_arm_with_boundaries1_expl_vde_adj_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int c_generated_code_static_robot_arm_with_boundaries1_expl_vde_adj_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void c_generated_code_static_robot_arm_with_boundaries1_expl_vde_adj_release(int mem) {
}

CASADI_SYMBOL_EXPORT void c_generated_code_static_robot_arm_with_boundaries1_expl_vde_adj_incref(void) {
}

CASADI_SYMBOL_EXPORT void c_generated_code_static_robot_arm_with_boundaries1_expl_vde_adj_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int c_generated_code_static_robot_arm_with_boundaries1_expl_vde_adj_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int c_generated_code_static_robot_arm_with_boundaries1_expl_vde_adj_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real c_generated_code_static_robot_arm_with_boundaries1_expl_vde_adj_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* c_generated_code_static_robot_arm_with_boundaries1_expl_vde_adj_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* c_generated_code_static_robot_arm_with_boundaries1_expl_vde_adj_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* c_generated_code_static_robot_arm_with_boundaries1_expl_vde_adj_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    case 2: return casadi_s1;
    case 3: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* c_generated_code_static_robot_arm_with_boundaries1_expl_vde_adj_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int c_generated_code_static_robot_arm_with_boundaries1_expl_vde_adj_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 4;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
