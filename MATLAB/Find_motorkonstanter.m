MX28Constants1 = MX28(1:34,2:3)\MX28(1:34,1);
MX28Constants2 = MX28(72:90,2:3)\MX28(72:90,1);

MX64Constants1 = MX64(1:118,2:3)\MX64(1:118,1);
MX64Constants2 = MX64(184:200,2:3)\MX64(184:200,1);

MX106Constants1 = MX106(1:78,2:3)\MX106(1:78,1);
MX106Constants2 = MX106(110:200,2:3)\MX106(110:200,1);

MX28TorqueConstant = [ MX28Constants1(2,1),(MX28Constants1(2,1)+MX28Constants2(2,1))/2,MX28Constants2(2,1) ]
MX28VelocityConstant = (MX28Constants1(1,1)+MX28Constants2(1,1))/2

MX64TorqueConstant = [ MX64Constants1(2,1),(MX64Constants1(2,1)+MX64Constants2(2,1))/2,MX64Constants2(2,1) ]
MX64VelocityConstant = (MX64Constants1(1,1)+MX64Constants2(1,1))/2 

MX106TorqueConstant = [ MX106Constants1(2,1),(MX106Constants1(2,1)+MX106Constants2(2,1))/2,MX106Constants2(2,1) ]
MX106VelocityConstant = (MX106Constants1(1,1)+MX106Constants2(1,1))/2