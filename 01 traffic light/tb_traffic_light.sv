`timescale 1ns/1ps
module tb_traffic_light;
    reg        clk, rstn, vs;
    wire [5:0] light;
    
    traffic_light u0 (
        .clk(clk), .rstn(rstn), .vs(vs), .light(light)
    );
    
    initial clk = 0;
    always #5 clk = ~clk;
    
    initial begin
        $dumpfile("dump.vcd");
        $dumpvars(0, tb_traffic_light);
        
        // 초기화 + Reset
        rstn = 0; vs = 0;
        #15;
        rstn = 1;
        
        // === 케이스 1: 주도로 녹색, 차 없음 → 계속 유지 ===
        // st00에서 vs=0이면 25초 지나도 안 나감
        vs = 0;
        #400;                  // 25클럭 훨씬 넘게 → st00 계속 (녹색 유지)
        
        // === 케이스 2: 부도로에 차 등장 → 주도로 녹색 종료 ===
        vs = 1;
        #100;                  // 25초 이미 지났으니 → st01(주도로 황) 전이
                               // → st10(부도로 녹)
        
        // === 케이스 3: 부도로 녹색 중 차 빠짐 → 조기 종료 ===
        // st10에서 vs=0 되면 25초 안 기다리고 바로 st11
        vs = 0;
        #100;                  // 부도로 녹색 빨리 끝남 → st11(부도로 황)
        
        // === 케이스 4: 한 바퀴 더 (차 계속 있는 상태) ===
        vs = 1;
        #500;                  // st00→01→10→11→00 정상 순환
        
        // === 케이스 5: 부도로 녹색 중 차 계속 있음 → 25초 꽉 채움 ===
        // st10에서 vs=1 유지 → tg(25초)로 종료
        vs = 1;
        #400;
        
        // === 케이스 6: Reset 중간에 ===
        rstn = 0;
        #20;
        rstn = 1;
        vs = 1;
        #200;
        
        $display("=== Simulation End ===");
        $finish;
    end
    
    initial begin
        $monitor("Time=%0t | rstn=%b vs=%b | st=%b cnt=%d | light=%b", 
                 $time, rstn, vs, u0.st, u0.cnt, light);
    end
endmodule
