module vga_demo(CLOCK_50, SW, KEY, LEDR, VGA_R, VGA_G, VGA_B,
                VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N, VGA_CLK,
                HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, GPIO_0);
    
    // Parameters defined inside the module body
    parameter nX = 10;
    parameter nY = 9;
    parameter A = 2'b00, B = 2'b01, C = 2'b10;

    // Port Declarations
    input CLOCK_50;    
    input [9:0] SW;
    input [3:0] KEY;
    output [9:0] LEDR;
    output [7:0] VGA_R;
    output [7:0] VGA_G;
    output [7:0] VGA_B;
    output VGA_HS;
    output VGA_VS;
    output VGA_BLANK_N;
    output VGA_SYNC_N;
    output VGA_CLK;
    output [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;
    inout [35:0] GPIO_0;

    // Wire and Reg Declarations
    wire UART_RX;
    assign UART_RX = GPIO_0[0]; // UART RX from GPIO pin
    wire [nX-1:0] dino_x, obstacle_x;
    wire [nY-1:0] dino_y, obstacle_y;
    wire [8:0] dino_color, obstacle_color;
    wire dino_write, obstacle_write;
    reg [nX-1:0] MUX_x;
    reg [nY-1:0] MUX_y;
    reg [8:0] MUX_color;
    reg MUX_write;
    wire req_dino, req_obstacle;
    reg gnt_dino, gnt_obstacle;
    reg [1:0] y_Q, Y_D;
    
    wire Resetn, jump_trigger_key, jump_trigger;
    wire collision, collision_latched;
    wire [15:0] score;
    wire [15:0] high_score; // FOR KEEPING TRACK OF HIGH SCORE
    wire respawn_obstacle;
    wire [7:0] speed_level;

    assign Resetn = KEY[0];
    sync S1 (~KEY[1], Resetn, CLOCK_50, jump_trigger_key);

    // --UART recieve + decode ---
    // -- UART receive with 16× oversampling (robust to baud skew) --
    wire        tick_16x;
    baud16x #(.CLK_HZ(50_000_000), .BAUD(115200)) U_BAUD16X (
        .clk     (CLOCK_50),
        .rst_n   (Resetn),       // Resetn=1 -> run; Resetn=0 -> reset
        .tick_16x(tick_16x)
    );

    wire        rx_ready;
    wire [7:0]  rx_byte;
    wire        jump_pulse, duck_pulse, idle_pulse;
    assign LEDR[1] = rx_ready;
    assign LEDR[2] = jump_pulse;
    assign LEDR[3] = duck_pulse;
    assign LEDR[4] = idle_pulse;

    assign LEDR[9] = Resetn; // off -> fpga is in reset (key[0] not pressed)
    // on -> FPGA is running normally

    uart_rx_16x #(.CLK_HZ(50_000_000), .BAUD(115200)) U_RX (
        .clk       (CLOCK_50),
        .rst_n     (Resetn),
        .os_tick   (tick_16x),
        .rx_raw    (UART_RX),    // SH-U09C TXD → this pin
        .data_ready(rx_ready),
        .data      (rx_byte)
    );

    uart_cmd_decoder U_DEC (
        .clk       (CLOCK_50),
        .rst_n     (Resetn),
        .rx_strobe (rx_ready),
        .rx_data   (rx_byte),
        .jump_pulse(jump_pulse),   // 1-cycle on 'J'
        .duck_pulse(duck_pulse),   // 1-cycle on 'D'
        .idle_pulse(idle_pulse)    // 1-cycle on 'I'
    );

    // Combine keyboard jump + UART jump
    assign jump_trigger = jump_trigger_key | jump_pulse;


    // FSM for arbitration between dinosaur and obstacle drawing
    always @ (*)
        case (y_Q)
            A:  if (req_dino) Y_D = B;
                else if (req_obstacle) Y_D = C;
                else Y_D = A;
            B:  if (req_dino) Y_D = B;
                else Y_D = A;
            C:  if (req_obstacle) Y_D = C;
                else Y_D = A;
            default: Y_D = A;
        endcase

    // MUX outputs for VGA
    always @ (*)
    begin
        gnt_dino = 1'b0; 
        gnt_obstacle = 1'b0; 
        MUX_write = 1'b0;
        MUX_x = dino_x; 
        MUX_y = dino_y; 
        MUX_color = dino_color;

        case (y_Q)
            A: ; // idle
            B: begin 
                gnt_dino = 1'b1; 
                MUX_write = dino_write; 
                MUX_x = dino_x; 
                MUX_y = dino_y; 
                MUX_color = dino_color; 
            end
            C: begin 
                gnt_obstacle = 1'b1;
                MUX_write = obstacle_write; 
                MUX_x = obstacle_x; 
                MUX_y = obstacle_y; 
                MUX_color = obstacle_color; 
            end
        endcase
    end

    always @(posedge CLOCK_50)
        if (Resetn == 0)
            y_Q <= A;
        else
            y_Q <= Y_D;

    // Instantiate dinosaur (player) - MODE 1 (Jumper)
    object DINO (
        .Resetn(Resetn && !collision_latched), 
        .Clock(CLOCK_50),
        .gnt(gnt_dino),         
        .sel(1'b1),
        .jump_trigger(jump_trigger),
        .new_color(9'b000111000),  // Green
        .faster(1'b0),
        .slower(1'b0),
        .req(req_dino), 
        .VGA_x(dino_x), 
        .VGA_y(dino_y), 
        .VGA_color(dino_color), 
        .VGA_write(dino_write)
    );
    defparam DINO.nX = nX;
    defparam DINO.nY = nY;
    defparam DINO.XSCREEN = 640;
    defparam DINO.YSCREEN = 480;
    defparam DINO.MODE = 1;
    defparam DINO.xOBJ = 5;
    defparam DINO.yOBJ = 5;
    defparam DINO.HAS_SPRITE = 1;
    defparam DINO.INIT_FILE = "./MIF/dino_sprite.mif";
    defparam DINO.X_INIT = 10'd104;
    defparam DINO.Y_INIT = 9'd269;
    defparam DINO.JUMP_HEIGHT = 9'd80;
    defparam DINO.KK = 19;

    // Instantiate obstacle - MODE 0 (Obstacle)
    object OBS (
        .Resetn(Resetn && !collision_latched), 
        .Clock(CLOCK_50), 
        .gnt(gnt_obstacle),
        .sel(1'b0),
        .jump_trigger(1'b0),
        .new_color(9'b111000000),  // Red
        .faster(respawn_obstacle),
        .slower(1'b0),
        .req(req_obstacle), 
        .VGA_x(obstacle_x), 
        .VGA_y(obstacle_y), 
        .VGA_color(obstacle_color), 
        .VGA_write(obstacle_write),
        .speed_level(speed_level)
    );
    defparam OBS.nX = nX;
    defparam OBS.nY = nY;
    defparam OBS.XSCREEN = 640;
    defparam OBS.YSCREEN = 480;
    defparam OBS.MODE = 0;
    defparam OBS.xOBJ = 5;
    defparam OBS.yOBJ = 5;
    defparam OBS.HAS_SPRITE = 1;
    defparam OBS.INIT_FILE = "./MIF/obstacle_sprite.mif";
    defparam OBS.X_INIT = 10'd104;
    defparam OBS.Y_INIT = 9'd285;
    defparam OBS.KK = 19;

    // VGA controller with dynamic background
    vga_adapter VGA (
        .resetn(Resetn),
        .clock(CLOCK_50),
        .color(MUX_color),
        .x(MUX_x),
        .y(MUX_y),
        .write(MUX_write),
        .VGA_R(VGA_R),
        .VGA_G(VGA_G),
        .VGA_B(VGA_B),
        .VGA_HS(VGA_HS),
        .VGA_VS(VGA_VS),
        .VGA_BLANK_N(VGA_BLANK_N),
        .VGA_SYNC_N(VGA_SYNC_N),
        .VGA_CLK(VGA_CLK)
    );
    
    // Switch background based on game state
    defparam VGA.BACKGROUND_IMAGE ="./MIF/bmp_640_9.mif";

    // Enhanced collision detection with proper bounding box
    assign collision = ((dino_x + 5 >= obstacle_x) && 
                       (dino_x <= obstacle_x + 11) &&
                       (dino_y + 5 >= obstacle_y) && 
                       (dino_y <= obstacle_y + 11));
    
    // Collision latch - stays high once collision detected
    collision_latch COL_LATCH (
        .Clock(CLOCK_50),
        .Resetn(Resetn),
        .collision(collision),
        .collision_latched(collision_latched)
    );
    
    // Score module with collision and respawn handling
    score_counter SCORE (
        .Clock(CLOCK_50),
        .Resetn(Resetn),
        .dino_x(dino_x),
        .obstacle_x(obstacle_x),
        .collision(collision),
        .collision_latched(collision_latched),
        .score(score),
        .high_score(high_score), // FOR HIGH SCORe
        .respawn_obstacle(respawn_obstacle),
        .speed_level(speed_level)
    );
    
    // Display current game score on HEX displays 0-2, and high score on 3-5
    wire [3:0] ones, tens, hundreds;
    wire [3:0] high_ones, high_tens, high_hundreds;

    assign ones = score % 10;
    assign tens = (score / 10) % 10;
    assign hundreds = (score / 100) % 10;

    assign high_ones = high_score % 10;
    assign high_tens = (high_score/10) % 10;
    assign high_hundreds = (high_score/100)%10;
    

    // Current Score Displays
    hex_display H0 (ones, HEX0);
    hex_display H1 (tens, HEX1);
    hex_display H2 (hundreds, HEX2);

    // High Score Displays
    hex_display H3(high_ones, HEX3);
    hex_display H4(high_tens, HEX4);
    hex_display H5(high_hundreds, HEX5);

    
    // LED indicators - latched collision for brightness
    assign LEDR[0] = collision_latched;


endmodule

// Collision Latch Module - keeps collision high once detected
module collision_latch(Clock, Resetn, collision, collision_latched);
    input Clock, Resetn;
    input collision;
    output reg collision_latched;
    
    always @(posedge Clock) begin
        if (!Resetn)
            collision_latched <= 1'b0;
        else if (collision)
            collision_latched <= 1'b1;
    end
endmodule

// Enhanced Score Counter Module with high score tracking
// Enhanced Score Counter Module with high score tracking
module score_counter(Clock, Resetn, dino_x, obstacle_x, collision, 
                     collision_latched, score, high_score, respawn_obstacle, speed_level);
    parameter nX = 10;
    
    input Clock, Resetn;
    input [nX-1:0] dino_x, obstacle_x;
    input collision, collision_latched;
    output reg [15:0] score;
    output reg [15:0] high_score;
    output reg respawn_obstacle;
    output reg [7:0] speed_level;
    
    reg prev_collision;
    reg prev_collision_latched;
    reg obstacle_passed;
    reg [nX-1:0] prev_obstacle_x;
    reg score_given;
    
    // Initialize high_score on first power-up only (not on reset)
    initial begin
        high_score = 16'd0;
    end
    
    // Detect when obstacle passes dino (moves from right to left past dino position)
    always @(posedge Clock) begin
        if (!Resetn) begin
            prev_obstacle_x <= 10'd640;
            obstacle_passed <= 1'b0;
            score_given <= 1'b0;
            prev_collision <= 1'b0;
            prev_collision_latched <= 1'b0;
            score <= 16'd0;
            respawn_obstacle <= 1'b0;
            speed_level <= 8'd0;
            // high_score is NOT reset here - it persists across resets!
        end
        else begin
            prev_obstacle_x <= obstacle_x;
            prev_collision <= collision;
            prev_collision_latched <= collision_latched;
            respawn_obstacle <= 1'b0;  // Default
            
            // Update high score when collision happens (game over)
            if (collision_latched && !prev_collision_latched) begin
                if (score > high_score) begin
                    high_score <= score;
                end
            end
            
            // Reset score when starting new game (collision_latched goes from 1 to 0)
            if (!collision_latched && prev_collision_latched) begin
                score <= 16'd0;
                speed_level <= 8'd0;
            end
            
            // Only count score when game is active (not in collision state)
            if (!collision_latched) begin
                // Detect when obstacle crosses dino's X position (moving left)
                // Obstacle passed if it was to the right of dino and now is to the left
                if (prev_obstacle_x > dino_x && obstacle_x <= dino_x && !score_given) begin
                    // Check if there was NO collision during the pass
                    if (!collision && !prev_collision) begin
                        score <= score + 16'd1;
                        speed_level <= speed_level + 8'd1;
                        respawn_obstacle <= 1'b1;  // Signal to respawn
                    end
                    score_given <= 1'b1;
                end
                
                // Reset score_given flag when obstacle wraps around
                if (obstacle_x > 10'd500 && prev_obstacle_x < 10'd100) begin
                    score_given <= 1'b0;
                end
            end
        end
    end
endmodule

// Hex Display Decoder
module hex_display(value, display);
    input [3:0] value;
    output reg [6:0] display;
    
    always @(*) begin
        case (value)
            4'h0: display = 7'b1000000; // 0
            4'h1: display = 7'b1111001; // 1
            4'h2: display = 7'b0100100; // 2
            4'h3: display = 7'b0110000; // 3
            4'h4: display = 7'b0011001; // 4
            4'h5: display = 7'b0010010; // 5
            4'h6: display = 7'b0000010; // 6
            4'h7: display = 7'b1111000; // 7
            4'h8: display = 7'b0000000; // 8
            4'h9: display = 7'b0010000; // 9
            default: display = 7'b1111111; // Blank
        endcase
    end
endmodule

// Synchronizer module
module sync(D, Resetn, Clock, Q);
    input D;
    input Resetn, Clock;
    output reg Q;
    reg Qi;

    always @(posedge Clock)
        if (Resetn == 0) begin
            Qi <= 1'b0;
            Q <= 1'b0;
        end
        else begin
            Qi <= D;
            Q <= Qi;
        end
endmodule

// Register module
module regn(R, Resetn, E, Clock, Q);
    parameter n = 8;
    input [n-1:0] R;
    input Resetn, E, Clock;
    output reg [n-1:0] Q;

    always @(posedge Clock)
        if (Resetn == 0)
            Q <= 'b0;
        else if (E)
            Q <= R;
endmodule

// Up/Down counter
module upDn_count (R, Clock, Resetn, L, E, Dir, Q);
    parameter n = 8;
    input [n-1:0] R;
    input Clock, Resetn, E, L, Dir;
    output reg [n-1:0] Q;

    always @ (posedge Clock)
        if (Resetn == 0)
            Q <= 0;
        else if (L == 1)
            Q <= R;
        else if (E)
            if (Dir) 
                Q <= Q + 1'b1;
            else
                Q <= Q - 1'b1;
endmodule

// Up counter
module Up_count (Clock, Resetn, Q);
    parameter n = 8;
    input Clock, Resetn;
    output reg [n-1:0] Q;

    always @ (posedge Clock)
        if (Resetn == 0)
            Q <= 'b0;
        else 
            Q <= Q + 1'b1;
endmodule

// Universal object module - supports both jumping (MODE=1) and scrolling (MODE=0)
module object (Resetn, Clock, gnt, sel, jump_trigger, new_color, faster, slower, req,  
               VGA_x, VGA_y, VGA_color, VGA_write, speed_level);

    // Parameters
    parameter nX = 10;
    parameter nY = 9;
    parameter XSCREEN = 640;
    parameter YSCREEN = 480;
    parameter MODE = 0;
    
    parameter xOBJ = 5;
    parameter yOBJ = 5;
    parameter BOX_SIZE_X = 1 << xOBJ;
    parameter BOX_SIZE_Y = 1 << yOBJ;
    parameter HAS_SPRITE = 0;
    parameter INIT_FILE = "";
    
    parameter X_INIT = 10'd0;
    parameter Y_INIT = 9'd239;
    parameter JUMP_HEIGHT = 9'd80;
    parameter KK = 18;
    parameter MM = 8;
    
    parameter A = 4'b0000, B = 4'b0001, C = 4'b0010, D = 4'b0011,
              E = 4'b0100, F = 4'b0101, G = 4'b0110, H = 4'b0111,
              I = 4'b1000, J = 4'b1001, K = 4'b1010, L = 4'b1011;
              
    parameter Running = 2'b00, Ascending = 2'b01, Descending = 2'b10;

    // Ports
    input Resetn, Clock;
    input gnt;
    input sel;
    input jump_trigger;
    input faster, slower;
    input [8:0] new_color;
    input [7:0] speed_level;
    output reg req;
    output [nX-1:0] VGA_x;
    output [nY-1:0] VGA_y;
    output [8:0] VGA_color;
    output VGA_write;

    // Internal Wires and Regs
    reg [nX-1:0] X_reg;
    reg [nY-1:0] Y_reg;
    reg [nX-1:0] X_prev;
    reg [nY-1:0] Y_prev;
    reg prev_select;
    
    wire [xOBJ-1:0] XC;
    wire [yOBJ-1:0] YC;
    wire [8:0] color, sprite_pixel;
    wire [KK-1:0] slow; 
    
    reg Lx, Ly, Lxc, Lyc, Exc, Eyc;
    reg erase, write;
    reg [3:0] y_Q, Y_D;
    
    reg [1:0] Jump_Q, Jump_D;
    reg jump_latch;
    
    reg [2:0] ys_Q, Ys_D;
    reg sll, srl;
    reg [MM-1:0] mask;
    
    // Random position generation for respawn
    reg [9:0] random_x_offset;
    wire [15:0] lfsr_out;
    
    parameter ALT = 9'b111111111;
    wire sync;
    wire sync_adjusted;
    
    upDn_count U3 ({xOBJ{1'd0}}, Clock, Resetn, Lxc, Exc, 1'b1, XC);
        defparam U3.n = xOBJ;
    upDn_count U4 ({yOBJ{1'd0}}, Clock, Resetn, Lyc, Eyc, 1'b1, YC);
        defparam U4.n = yOBJ;
    
    Up_count U6 (Clock, Resetn, slow);
        defparam U6.n = KK;
    
    // LFSR for random number generation
    lfsr_16bit RAND_GEN (Clock, Resetn, lfsr_out);
    
    // Adjust speed based on speed_level
    wire [MM-1:0] speed_mask;
    assign speed_mask = (speed_level > 8'd10) ? 8'hFF : 
                       (speed_level > 8'd5) ? (mask | 8'h0F) : mask;
    
    assign sync = ((slow | (speed_mask << KK-MM)) == {KK{1'b1}});
    assign sync_adjusted = sync;
    
    // Respawn logic - when faster signal triggers
    always @(posedge Clock) begin
        if (!Resetn) begin
            random_x_offset <= 10'd0;
        end
        else if (MODE == 0 && faster) begin
            // Generate random X offset (0 to 300) for variety
            random_x_offset <= (lfsr_out[9:0] % 10'd300);
        end
    end
    
    always @(posedge Clock) begin
        if (Resetn == 0)
            jump_latch <= 1'b0;
        else if (MODE == 1) begin
            if (sel && jump_trigger && Jump_Q == Running)
                jump_latch <= 1'b1;
            else if (Jump_Q == Ascending)
                jump_latch <= 1'b0;
        end
    end
    
    always @(posedge Clock) begin
        if (Resetn == 0) begin
            X_prev <= X_INIT;
            Y_prev <= Y_INIT;
        end
        else if (sync_adjusted) begin
            X_prev <= X_reg;
            Y_prev <= Y_reg;
        end
    end

    always @(posedge Clock) begin
        if (Resetn == 0) 
            X_reg <= X_INIT;
        else if (Lx) 
            X_reg <= X_INIT;
        else if (MODE == 0) begin
            if (faster) begin
                // Respawn at right edge with random offset
                X_reg <= XSCREEN - BOX_SIZE_X + random_x_offset;
            end
            else if (sync_adjusted) begin
                if (X_reg <= 1)
                    X_reg <= XSCREEN - BOX_SIZE_X;
                else
                    X_reg <= X_reg - 1'b1;
            end
        end
    end
    
    always @(posedge Clock) begin
        if (Resetn == 0) 
            Y_reg <= Y_INIT;
        else if (Ly) 
            Y_reg <= Y_INIT;
        else if (MODE == 1 && sync_adjusted) begin
            case (Jump_Q)
                Ascending: begin
                    if (Y_reg > Y_INIT - JUMP_HEIGHT)
                        Y_reg <= Y_reg - 1'b1;
                end
                Descending: begin
                    if (Y_reg < Y_INIT)
                        Y_reg <= Y_reg + 1'b1;
                end
                default: ;
            endcase
        end
    end
    
    always @(*) begin
        Jump_D = Jump_Q;
        if (MODE == 1) begin
            case(Jump_Q)
                Running: begin
                    if (jump_latch) 
                        Jump_D = Ascending;
                end
                Ascending: begin
                    if (Y_reg <= Y_INIT - JUMP_HEIGHT) 
                        Jump_D = Descending;
                end
                Descending: begin
                    if (Y_reg >= Y_INIT) 
                        Jump_D = Running;
                end
                default: Jump_D = Running;
            endcase
        end
    end
    
    always @(posedge Clock)
        if (Resetn == 0) 
            Jump_Q <= Running;
        else 
            Jump_Q <= Jump_D;
    
    regn UC (new_color, Resetn, 1'b1, Clock, color);
        defparam UC.n = 9;

    wire [8:0] sprite_color;

    object_mem SPRITE_ROM(
        .address({YC,XC}),
        .clock(Clock),
        .q(sprite_pixel)
    );

    defparam SPRITE_ROM.n = 9;
    defparam SPRITE_ROM.Mn = xOBJ + yOBJ;
    defparam SPRITE_ROM.INIT_FILE = INIT_FILE;


    
    always @ (*)
        case (y_Q)
            A:  Y_D = B;
            B:  if (XC != BOX_SIZE_X-1) Y_D = B; else Y_D = C;
            C:  if (YC != BOX_SIZE_Y-1) Y_D = B; else Y_D = D;
            D:  if (!sync_adjusted) Y_D = D; else Y_D = E;
            E:  if (!gnt) Y_D = E; else Y_D = F;
            F:  if (XC != BOX_SIZE_X-1) Y_D = F; else Y_D = G;
            G:  if (YC != BOX_SIZE_Y-1) Y_D = F; else Y_D = H;
            H:  Y_D = I;
            I:  Y_D = J;
            J:  if (XC != BOX_SIZE_X-1) Y_D = J; else Y_D = K;
            K:  if (YC != BOX_SIZE_Y-1) Y_D = J; else Y_D = L;
            L:  Y_D = D;
            default: Y_D = A;
        endcase
    
    always @ (*)
    begin
        Lx = 1'b0; Ly = 1'b0; Lxc = 1'b0; Lyc = 1'b0; 
        Exc = 1'b0; Eyc = 1'b0;
        erase = 1'b0; write = 1'b0; req = 1'b0;
        prev_select = 1'b0;
        
        case (y_Q)
            A:  begin Lx = 1'b1; Ly = 1'b1; Lxc = 1'b1; Lyc = 1'b1; end
            B:  begin Exc = 1'b1; write = 1'b1; end
            C:  begin Lxc = 1'b1; Eyc = 1'b1; end
            D:  Lyc = 1'b1;
            E:  req = 1'b1;
            F:  begin req = 1'b1; Exc = 1'b1; erase = 1'b1; write = 1'b1; prev_select = 1'b1; end
            G:  begin req = 1'b1; Lxc = 1'b1; Eyc = 1'b1; prev_select = 1'b1; end
            H:  begin req = 1'b1; Lyc = 1'b1; end
            I:  begin req = 1'b1; end
            J:  begin req = 1'b1; Exc = 1'b1; write = 1'b1; end
            K:  begin req = 1'b1; Lxc = 1'b1; Eyc = 1'b1; end
            L:  Lyc = 1'b1;
        endcase
    end
    
    always @(posedge Clock)
        if (Resetn == 0)
            y_Q <= A;
        else
            y_Q <= Y_D;
    
    assign VGA_x = (MODE == 0) ? ((prev_select) ? X_prev : X_reg) + XC : X_INIT + XC;
    assign VGA_y = (MODE == 1) ? ((prev_select) ? Y_prev : Y_reg) + YC : Y_INIT + YC;

    assign VGA_color = erase ? ALT : (HAS_SPRITE ? sprite_pixel:color);
    assign VGA_write = write;
    
    always @(posedge Clock) begin
        if (Resetn == 0)
            mask <= 'b0;
        else if (srl) begin
            mask[MM-2:0] <= mask[MM-1:1];
            mask[MM-1] <= 1'b1;
        end
        else if (sll) begin
            mask[MM-1:1] <= mask[MM-2:0];
            mask[0] <= 1'b0;
        end
    end
    
    parameter As = 3'b000, Bs = 3'b001, Cs = 3'b010, Ds = 3'b011, Es = 3'b100;
    
    always @ (*)
        case (ys_Q)
            As: if (sel & faster) Ys_D = Bs;
                else if (sel & slower) Ys_D = Ds;
                else Ys_D = As;
            Bs: Ys_D = Cs;
            Cs: if (sel & faster) Ys_D = Cs; else Ys_D = As;
            Ds: Ys_D = Es;
            Es: if (sel & slower) Ys_D = Es; else Ys_D = As;
            default: Ys_D = As;
        endcase
    
    always @ (*)
    begin
        sll = 1'b0; srl = 1'b0;
        case (ys_Q)
            Bs: srl = 1'b1;
            Ds: sll = 1'b1;
            default: ;
        endcase
    end
       
    always @(posedge Clock)
        if (Resetn == 0)
            ys_Q <= As;
        else
            ys_Q <= Ys_D;

endmodule

// 16-bit Linear Feedback Shift Register for random number generation
module lfsr_16bit(Clock, Resetn, random_out);
    input Clock, Resetn;
    output reg [15:0] random_out;
    
    wire feedback;
    assign feedback = random_out[15] ^ random_out[14] ^ random_out[12] ^ random_out[3];
    
    always @(posedge Clock) begin
        if (!Resetn)
            random_out <= 16'hACE1; // Non-zero seed
        else
            random_out <= {random_out[14:0], feedback};
    end
endmodule


// ===== 16× baud tick from 50 MHz =====
module baud16x #(parameter CLK_HZ=50_000_000, BAUD=115200)(
  input  wire clk,
  input  wire rst_n,          // active-high run; 0=reset
  output reg  tick_16x        // 1-cycle pulse at BAUD*16
);
  localparam integer DIV = CLK_HZ/(BAUD*16); // ≈ 27 for 50MHz/115200
  reg [$clog2(DIV)-1:0] cnt;
  always @(posedge clk) begin
    if (!rst_n) begin cnt <= 0; tick_16x <= 1'b0; end
    else begin
      tick_16x <= 1'b0;
      if (cnt == DIV-1) begin cnt <= 0; tick_16x <= 1'b1; end
      else cnt <= cnt + 1'b1;
    end
  end
endmodule

// ===== UART RX with 16× oversampling (8-N-1) =====
module uart_rx_16x #(
  parameter CLK_HZ=50_000_000,
  parameter BAUD  =115200
)(
  input  wire       clk,
  input  wire       rst_n,       // active-high run; 0=reset
  input  wire       os_tick,     // 16× tick from baud16x
  input  wire       rx_raw,      // connect adapter TXD (idle HIGH)
  output reg        data_ready,  // 1 clk pulse when data valid
  output reg [7:0]  data
);
  // 2-FF synchronizer
  reg rx_m, rx_s;
  always @(posedge clk) begin
    rx_m <= rx_raw;
    rx_s <= rx_m;
  end

  localparam IDLE=2'd0, START=2'd1, DATA=2'd2, STOP=2'd3;
  reg [1:0] st;
  reg [3:0] os_cnt;   // 0..15
  reg [2:0] bit_i;    // 0..7
  reg [7:0] sh;

  always @(posedge clk) begin
    if (!rst_n) begin
      st         <= IDLE;
      os_cnt     <= 4'd0;
      bit_i      <= 3'd0;
      sh         <= 8'h00;
      data       <= 8'h00;
      data_ready <= 1'b0;
    end else begin
      data_ready <= 1'b0;

      if (os_tick) begin
        case (st)
          IDLE: if (rx_s==1'b0) begin st<=START; os_cnt<=4'd7; end
          START: if (os_cnt==0) begin
                   if (rx_s==1'b0) begin st<=DATA; os_cnt<=4'd15; bit_i<=3'd0; end
                   else st<=IDLE; // false start
                 end else os_cnt<=os_cnt-1'b1;
          DATA: if (os_cnt==0) begin
                  sh[bit_i] <= rx_s;   // sample center
                  os_cnt    <= 4'd15;
                  if (bit_i==3'd7) st<=STOP; else bit_i<=bit_i+1'b1;
                end else os_cnt<=os_cnt-1'b1;
          STOP: if (os_cnt==0) begin
                  data       <= sh;
                  data_ready <= 1'b1;
                  st         <= IDLE;
                end else os_cnt<=os_cnt-1'b1;
        endcase
      end
    end
  end
endmodule

// ===== 'J'/'D'/'I' byte decoder → 1-cycle pulses =====
module uart_cmd_decoder(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx_strobe,
    input  wire [7:0] rx_data,
    output reg        jump_pulse,
    output reg        duck_pulse,
    output reg        idle_pulse
);
    always @(posedge clk) begin
        if (!rst_n) begin
            jump_pulse <= 1'b0;
            duck_pulse <= 1'b0;
            idle_pulse <= 1'b0;
        end else begin
            jump_pulse <= 1'b0;
            duck_pulse <= 1'b0;
            idle_pulse <= 1'b0;
            if (rx_strobe) begin
                case (rx_data)
                    "J": jump_pulse <= 1'b1; // 0x4A
                    "D": duck_pulse <= 1'b1; // 0x44
                    "I": idle_pulse <= 1'b1; // 0x49
                    default: ;
                endcase
            end
        end
    end
endmodule

module object_mem (address, clock, q);
    parameter n = 3;    // memory width
    parameter Mn = 6;   // address bits
    parameter INIT_FILE = "./MIF/object_mem_8_8_3.mif";

	input wire [Mn-1:0] address;
	input wire clock;
	output [n-1:0]  q;
	wire [n-1:0] sub_wire0;
	wire [n-1:0] q = sub_wire0[n-1:0];

	altsyncram	altsyncram_component (
				.address_a (address),
				.clock0 (clock),
				.q_a (sub_wire0),
				.aclr0 (1'b0),
				.aclr1 (1'b0),
				.address_b (1'b1),
				.addressstall_a (1'b0),
				.addressstall_b (1'b0),
				.byteena_a (1'b1),
				.byteena_b (1'b1),
				.clock1 (1'b1),
				.clocken0 (1'b1),
				.clocken1 (1'b1),
				.clocken2 (1'b1),
				.clocken3 (1'b1),
				.data_a ({n{1'b1}}),
				.data_b (1'b1),
				.eccstatus (),
				.q_b (),
				.rden_a (1'b1),
				.rden_b (1'b1),
				.wren_a (1'b0),
				.wren_b (1'b0));
	defparam
		altsyncram_component.address_aclr_a = "NONE",
		altsyncram_component.clock_enable_input_a = "BYPASS",
		altsyncram_component.clock_enable_output_a = "BYPASS",
		altsyncram_component.init_file = INIT_FILE,
		altsyncram_component.intended_device_family = "Cyclone V",
		altsyncram_component.lpm_hint = "ENABLE_RUNTIME_MOD=NO",
		altsyncram_component.lpm_type = "altsyncram",
		altsyncram_component.numwords_a = 1 << Mn,
		altsyncram_component.operation_mode = "ROM",
		altsyncram_component.outdata_aclr_a = "NONE",
		altsyncram_component.outdata_reg_a = "UNREGISTERED",
		altsyncram_component.widthad_a = Mn,
		altsyncram_component.width_a = n,
		altsyncram_component.width_byteena_a = 1;
endmodule
