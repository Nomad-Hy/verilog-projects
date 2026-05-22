module traffic_light(input vs,input clk,input rstn, output reg [5:0]light);
  
  
  parameter st00=2'b00,st01=2'b01,st10=2'b10,st11=2'b11;
  parameter T_GREEN = 24;   // 25 cycle (0~24)
    parameter T_YELLOW = 3;   // 4 cycle  (0~3)
  reg[4:0] cnt;
  reg tg,tr;
  reg [1:0]st,nst;
  
  
  always @(posedge clk or negedge rstn)
    begin
      
      if(!rstn) begin st<=st00; cnt<=0;end
      else begin st<=nst;
      
      
      if(st!=nst)cnt<=0;
        else if (cnt==T_GREEN) cnt<=cnt;
      else cnt<=cnt+1;
      
      end
     
      
    end
  
  
  always @(*)
    begin
      
      nst=st;
      tg=(cnt==T_GREEN);
      tr=(cnt==T_YELLOW);
      
      case(st)
        st00:if(tg&&vs) nst=st01;
        st01:if(tr) nst=st10;
        st10:if(tg||(!vs&&cnt>=3)) nst=st11;
        st11:if(tr) nst=st00;
        default:nst=st00;
        
      endcase
      
    end
  
  
  always@(*)
    begin
      
      case(st)
        st00:light=6'b001100;
        st01:light=6'b010100;
        st10:light=6'b100001;
        st11:light=6'b100010;
        default:light=6'b000000;
      endcase
    end
  
  
endmodule
