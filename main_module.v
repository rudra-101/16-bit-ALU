`timescale 1ns / 1ps

//// 16-bit ALU Design Using Verilog.

////////////////////// Main Module /////////////////////

module main_module(y1,y2,y3,cout,carry_out,sel,a,b,x,y,A,B,j,k,cin,Op);

output reg [15:0]y1;
output reg [31:0]y2;
output reg [15:0]y3;
output cout;
output carry_out;
input [1:0]sel;
input [15:0]a,b,x,y,A,B,j,k;
input cin;
input Op;

wire [15:0]s,sum,rightshift,leftshift;
wire [31:0]P;

///////////////////////////////////////////

CLAdder c1(s,cout,a,b,cin);
subtractor sub1(sum,carry_out,x,y,Op);
multiplier m1(A,B,P);
shifter shift1(leftshift,rightshift,j,k);
 

always@*
    begin
        if(sel==2'b00)
        begin
             y1[15:0] = s[15:0];
             y2[31:0] = 32'bz;
             y3[15:0] = 16'bz; 
        end
        else if(sel==2'b01)
        begin
             y1[15:0] = sum[15:0];
             y2 [31:0] = 32'bz;
             y3[15:0] = 16'bz;  
        end
        else if(sel==2'b10)
        begin
             y2[31:0] = P;   
             y1[15:0] = 16'bz;
             y3[15:0] = 16'bz;
        end
        else if(sel==2'b11)
        begin   
             y3[15:0] = leftshift[15:0];
             y1[15:0] = rightshift[15:0];
             y2 [31:0] = 32'bz;
        end
    end
endmodule
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////// ADDER //////////////////////
module CLAdder(s, cout, a,b, cin);
output [15:0]s;
output cout;
input cin;

input [15:0]a,b;

wire cout1,cout2,cout3;                                 //Final Carry out of 16 bit CLA
cladder4 cla1(s[3:0],cout1,a[3:0],b[3:0],cin);          //cladder4 function is called
cladder4 cla2(s[7:4],cout2,a[7:4],b[7:4],cout1);        //cladder4 function is called     
cladder4 cla3(s[11:8],cout3,a[11:8],b[11:8],cout2);        //cladder4 function is called
cladder4 cla4(s[15:12],cout,a[15:12],b[15:12],cout3);        //cladder4 function is called
endmodule
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
module cladder4(output [3:0]s, c3, input [3:0]a, [3:0]b, cin);

wire c0,p0,g0,c1,p1,g1,c2,p2,g2,p3,g3;

PFAdder pfa1(s[0],p0,g0,a[0],b[0],cin);               //Partial adder function is called
cla_logic clogic1(c0,p0,g0,cin);                  //Propagate and generate Carry function is called

PFAdder pfa2(s[1],p1,g1,a[1],b[1],c0);
cla_logic clogic2(c1,p1,g1,c0);

PFAdder pfa3(s[2],p2,g2,a[2],b[2],c1);
cla_logic clogic3(c2,p2,g2,c1);

PFAdder pfa4(s[3],p3,g3,a[3],b[3],c2);
cla_logic clogic4(c3,p3,g3,c2);
endmodule
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

module PFAdder(output s,p,g, input a,b,cin);
xor(s,a,b,cin);                        //Sum is calculated
and(g,a,b);                            //Generate carry is calaculated
xor(p,a,b);                           //Propagate carry is calculated    
endmodule
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////CLA Logic:

module cla_logic(output c,input p,g,cin);
wire w1;                     //Wire for intermediate input
and(w1,p,cin);              //Propagate carry logic
or(c,g,w1);                 //Generate carry logic
endmodule

/////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////// SUBTRACTOR ///////////////////////
module subtractor (sum, carry_out, x, y, Op);
  input [15:0] x, y;
  input Op;
  output [15:0] sum;
  output carry_out;
  
  wire [15:0] B_inverted;
  wire [15:0] C;

  // Invert B based on Op
  assign B_inverted = (Op) ? ~y : y;

  // Full Adder 0
   fulladder FA0 (sum[0],C[0],x[0],B_inverted[0],Op);
   genvar i;
   generate
       for(i=1;i<16;i=i+1)
           begin
               fulladder FAi(sum[i],C[i],x[i],B_inverted[i],C[i-1]);
           end
   endgenerate

endmodule

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

module fulladder(S, Cout, A, B, Cin);
   output S,Cout;
   input  A,B,Cin;
     wire   w1,w2,w3;
   xor(S, Cin, A,B);
   xor(w1, A, B);  
   and(w2, w1, Cin);
   and(w3, A, B);  
   or(Cout, w2, w3);
endmodule
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////// MULTIPLIER ////////////////////////

module multiplier (A,B,P);
  input [15:0]A,B;
  output reg [31:0] P;
  reg [15:0] B_reg;
  reg [4:0] counter;

  always @(A or B) begin
    P <= 0;
    B_reg <= B;
    counter <= 0;
  end

  always @(*) 
   begin
      if (counter < 16) begin
        if (B_reg[0] == 1'b1)
          P <= P + (A << counter);
        B_reg <= B_reg >> 1;
        counter <= counter + 1;
      end
    end

endmodule

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////// SHIFTER /////////////////
module shifter(
    output reg [15:0] leftshift,rightshift,
    input [15:0] j,k);
always@(k)
begin
leftshift <= j<<k;
rightshift <= j>>k;
end

endmodule
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
