module vga_demo(CLOCK_50, SW, KEY, LEDR, VGA_R, VGA_G, VGA_B,
	VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N, VGA_CLK,
	HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, GPIO_0);

	/*
	 * GAMEOVER MIF FILE REQUIREMENTS:
	 * 
	 * Each gameover MIF file must be:
	 * - Exactly 128x64 pixels (8,192 total pixels = 2^13, perfect power of 2!)
	 * - 9-bit color format (3 bits R, 3 bits G, 3 bits B)
	 * - Stored LINEARLY (address = row * 128 + col)
	 * - NO TRANSPARENCY - all pixels are drawn exactly as they are
	 * 
	 * Required MIF format:
	 * DEPTH = 8192;      <-- Exactly 128 * 64 = 8192
	 * WIDTH = 9;
	 * ADDRESS_RADIX = DEC;
	 * DATA_RADIX = BIN;
	 * CONTENT
	 * BEGIN
	 * 0 : <any color>;       -- All colors are drawn (no transparency)
	 * 1 : <any color>;
	 * ...
	 * 8191 : <any color>;    -- Last pixel
	 * END;
	 * 
	 * Address calculation: address = row * 128 + col
	 * - Row 0: addresses 0-127
	 * - Row 1: addresses 128-255
	 * - Row 2: addresses 256-383
	 * - ...
	 * - Row 63: addresses 8064-8191
	 */

	// Parameters defined inside the module body - UPDATED FOR 320x240
	parameter nX = 9;  // Changed from 10 to 9 bits for 320 width
	parameter nY = 8;  // Changed from 9 to 8 bits for 240 height
	parameter A = 3'b000, B = 3'b001, C = 3'b010, D = 3'b011, E = 3'b100, F = 3'b101, G = 3'b110;

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

	// Wire and Reg Declarations - UPDATED BIT WIDTHS
	wire UART_RX;
	assign UART_RX = GPIO_0[0]; // UART RX from GPIO pin
	wire [8:0] dino_base_x, obstacle_base_x;      // Changed from [9:0] to [8:0]
    wire [7:0] dino_base_y, obstacle_base_y;      // Changed from [8:0] to [7:0]
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
	reg [2:0] y_Q, Y_D;

	// Game Over object wires
	wire [nX-1:0] gameover1_x, gameover2_x, gameover3_x, gameover4_x;
	wire [nY-1:0] gameover1_y, gameover2_y, gameover3_y, gameover4_y;
	wire [8:0] gameover1_color, gameover2_color, gameover3_color, gameover4_color;
	wire gameover1_write, gameover2_write, gameover3_write, gameover4_write;
	wire req_gameover1, req_gameover2, req_gameover3, req_gameover4;
	reg gnt_gameover1, gnt_gameover2, gnt_gameover3, gnt_gameover4;

	wire Resetn, jump_trigger_key, jump_trigger;
	wire collision, collision_latched;
	wire [15:0] score;
	wire [15:0] high_score;
	wire respawn_obstacle;
	wire [7:0] speed_level;
	wire duck_trigger_key;

	assign Resetn = KEY[0];
	sync S1 (~KEY[1], Resetn, CLOCK_50, jump_trigger_key);
	sync S2(~KEY[2], Resetn, CLOCK_50, duck_trigger_key);

	// UART "D" command and physical key[2]
	wire duck_combined;
	assign duck_combined = duck_pulse | duck_trigger_key;



	// --UART recieve + decode ---
	wire tick_16x;
	baud16x #(.CLK_HZ(50_000_000), .BAUD(115200)) U_BAUD16X (
		.clk (CLOCK_50),
		.rst_n (Resetn),
		.tick_16x(tick_16x)
	);

	wire rx_ready;
	wire [7:0] rx_byte;
	wire jump_pulse, duck_pulse, idle_pulse;
	assign LEDR[1] = rx_ready;
	assign LEDR[2] = jump_pulse;
	assign LEDR[3] = duck_pulse;
	assign LEDR[4] = idle_pulse;
	assign LEDR[9] = Resetn;

	uart_rx_16x #(.CLK_HZ(50_000_000), .BAUD(115200)) U_RX (
		.clk (CLOCK_50),
		.rst_n (Resetn),
		.os_tick (tick_16x),
		.rx_raw (UART_RX),
		.data_ready(rx_ready),
		.data (rx_byte)
	);

	uart_cmd_decoder U_DEC (
		.clk (CLOCK_50),
		.rst_n (Resetn),
		.rx_strobe (rx_ready),
		.rx_data (rx_byte),
		.jump_pulse(jump_pulse),
		.duck_pulse(duck_pulse),
		.idle_pulse(idle_pulse)
	);

	assign jump_trigger = jump_trigger_key | jump_pulse;

	// Determine which gameover image to show based on score
	wire show_gameover1, show_gameover2, show_gameover3, show_gameover4;
	assign show_gameover1 = collision_latched && (score < 16'd10);
	assign show_gameover2 = collision_latched && (score >= 16'd10 && score < 16'd15);
	assign show_gameover3 = collision_latched && (score >= 16'd15 && score < 16'd20);
	assign show_gameover4 = collision_latched && (score >= 16'd20);

	// FSM for arbitration between dinosaur and obstacle drawing
	always @ (*)
	case (y_Q)
		A: if (req_dino) Y_D = B;
		   else if (req_obstacle) Y_D = C;
		   else if (show_gameover1 && req_gameover1) Y_D = D;
		   else if (show_gameover2 && req_gameover2) Y_D = E;
		   else if (show_gameover3 && req_gameover3) Y_D = F;
		   else if (show_gameover4 && req_gameover4) Y_D = G;
		   else Y_D = A;
		B: if (req_dino) Y_D = B;
		   else Y_D = A;
		C: if (req_obstacle) Y_D = C;
		   else Y_D = A;
		D: if (show_gameover1 && req_gameover1) Y_D = D;
		   else Y_D = A;
		E: if (show_gameover2 && req_gameover2) Y_D = E;
		   else Y_D = A;
		F: if (show_gameover3 && req_gameover3) Y_D = F;
		   else Y_D = A;
		G: if (show_gameover4 && req_gameover4) Y_D = G;
		   else Y_D = A;
		default: Y_D = A;
	endcase

	// MUX outputs for VGA
	always @ (*)
	begin
		gnt_dino = 1'b0;
		gnt_obstacle = 1'b0;
		gnt_gameover1 = 1'b0;
		gnt_gameover2 = 1'b0;
		gnt_gameover3 = 1'b0;
		gnt_gameover4 = 1'b0;
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
			D: begin
				gnt_gameover1 = 1'b1;
				MUX_write = gameover1_write;
				MUX_x = gameover1_x;
				MUX_y = gameover1_y;
				MUX_color = gameover1_color;
			end
			E: begin
				gnt_gameover2 = 1'b1;
				MUX_write = gameover2_write;
				MUX_x = gameover2_x;
				MUX_y = gameover2_y;
				MUX_color = gameover2_color;
			end
			F: begin
				gnt_gameover3 = 1'b1;
				MUX_write = gameover3_write;
				MUX_x = gameover3_x;
				MUX_y = gameover3_y;
				MUX_color = gameover3_color;
			end
			G: begin
				gnt_gameover4 = 1'b1;
				MUX_write = gameover4_write;
				MUX_x = gameover4_x;
				MUX_y = gameover4_y;
				MUX_color = gameover4_color;
			end
		endcase
	end

	always @(posedge CLOCK_50)
	if (Resetn == 0)
		y_Q <= A;
	else
		y_Q <= Y_D;

	// Instantiate dinosaur (player) - MODE 1 (Jumper) - SCALED POSITIONS
	object DINO (
		.Resetn(Resetn && !collision_latched),
		.Clock(CLOCK_50),
		.gnt(gnt_dino),
		.sel(1'b1),
		.jump_trigger(jump_trigger),
		.duck_trigger(duck_combined),    
		.new_color(9'b000111000),
		.faster(1'b0),
		.slower(1'b0),
		.speed_level(speed_level),    
		.req(req_dino),
		.VGA_x(dino_x),
		.VGA_y(dino_y),
		.VGA_color(dino_color),
		.VGA_write(dino_write),
		.BASE_X(dino_base_x), 
        .BASE_Y(dino_base_y)
	);
	defparam DINO.nX = nX;
	defparam DINO.nY = nY;
	defparam DINO.XSCREEN = 320;     // Changed from 640
	defparam DINO.YSCREEN = 240;     // Changed from 480
	defparam DINO.MODE = 1;
	defparam DINO.xOBJ = 5;
	defparam DINO.yOBJ = 5;
	defparam DINO.HAS_SPRITE = 1;
	defparam DINO.X_INIT = 9'd52;    // Changed from 10'd104 (halved)
	defparam DINO.Y_INIT = 8'd109;   // Changed from 9'd258 (halved)
	defparam DINO.JUMP_HEIGHT = 8'd60; // Changed from 9'd80 (halved)
	defparam DINO.KK = 21;

	// Instantiate obstacle - MODE 0 (Obstacle) - SCALED POSITIONS
	object OBS (
		.Resetn(Resetn && !collision_latched),
		.Clock(CLOCK_50),
		.gnt(gnt_obstacle),
		.sel(1'b0),
		.jump_trigger(1'b0),
        .duck_trigger(1'b0),         
		.new_color(9'b111000000),
		.faster(respawn_obstacle),
		.slower(1'b0),
        .speed_level(speed_level),   
		.req(req_obstacle),
		.VGA_x(obstacle_x),
		.VGA_y(obstacle_y),
		.VGA_color(obstacle_color),
		.VGA_write(obstacle_write),
		.BASE_X(obstacle_base_x), 
   	    .BASE_Y(obstacle_base_y)
	);
	defparam OBS.nX = nX;
	defparam OBS.nY = nY;
	defparam OBS.XSCREEN = 320;      // Changed from 640
	defparam OBS.YSCREEN = 240;      // Changed from 480
	defparam OBS.MODE = 0;
	defparam OBS.xOBJ = 4;
	defparam OBS.yOBJ = 4;
	defparam OBS.HAS_SPRITE = 1;
	defparam OBS.INIT_FILE = "./MIF/leetcodeObstacle.mif";
	defparam OBS.X_INIT = 9'd320;    // Changed from 10'd640 (halved)
	defparam OBS.Y_INIT = 8'd109;    // Changed from 9'd269 (approximately halved)
	defparam OBS.KK = 19;

	// Instantiate Game Over objects - Centered and stationary
	// Screen: 320x240, Image: 128x64
	// Center X = (320-128)/2 = 96, Center Y = (240-64)/2 = 88
	
	// Game Over 1: Score < 10
	object GAMEOVER1 (
		.Resetn(Resetn),  // Always keep active, control via FSM
		.Clock(CLOCK_50),
		.gnt(gnt_gameover1),
		.sel(1'b0),
		.jump_trigger(1'b0),
		.duck_trigger(1'b0),
		.new_color(9'b000000000),
		.faster(1'b0),
		.slower(1'b0),
		.speed_level(8'd0),
		.req(req_gameover1),
		.VGA_x(gameover1_x),
		.VGA_y(gameover1_y),
		.VGA_color(gameover1_color),
		.VGA_write(gameover1_write),
		.BASE_X(),
		.BASE_Y()
	);
	defparam GAMEOVER1.nX = nX;
	defparam GAMEOVER1.nY = nY;
	defparam GAMEOVER1.XSCREEN = 320;
	defparam GAMEOVER1.YSCREEN = 240;
	defparam GAMEOVER1.MODE = 2;  // New MODE for gameover (stationary, always requests)
	defparam GAMEOVER1.xOBJ = 7;  // 2^7 = 128 (exact width)
	defparam GAMEOVER1.yOBJ = 6;  // 2^6 = 64 (exact height)
	defparam GAMEOVER1.GAMEOVER_WIDTH = 128;  // Actual image width
	defparam GAMEOVER1.GAMEOVER_HEIGHT = 64;  // Actual image height
	defparam GAMEOVER1.HAS_SPRITE = 1;
	defparam GAMEOVER1.STATIONARY = 1;
	defparam GAMEOVER1.INIT_FILE = "./MIF/gameover1.mif";
	defparam GAMEOVER1.X_INIT = 9'd96;  // Centered
	defparam GAMEOVER1.Y_INIT = 8'd88;  // Centered
	defparam GAMEOVER1.KK = 19;

	// Game Over 2: Score 10-14
	object GAMEOVER2 (
		.Resetn(Resetn),
		.Clock(CLOCK_50),
		.gnt(gnt_gameover2),
		.sel(1'b0),
		.jump_trigger(1'b0),
		.duck_trigger(1'b0),
		.new_color(9'b000000000),
		.faster(1'b0),
		.slower(1'b0),
		.speed_level(8'd0),
		.req(req_gameover2),
		.VGA_x(gameover2_x),
		.VGA_y(gameover2_y),
		.VGA_color(gameover2_color),
		.VGA_write(gameover2_write),
		.BASE_X(),
		.BASE_Y()
	);
	defparam GAMEOVER2.nX = nX;
	defparam GAMEOVER2.nY = nY;
	defparam GAMEOVER2.XSCREEN = 320;
	defparam GAMEOVER2.YSCREEN = 240;
	defparam GAMEOVER2.MODE = 2;
	defparam GAMEOVER2.xOBJ = 8;  // 2^8 = 256
	defparam GAMEOVER2.yOBJ = 7;  // 2^7 = 128
	defparam GAMEOVER2.GAMEOVER_WIDTH = 150;
	defparam GAMEOVER2.GAMEOVER_HEIGHT = 75;
	defparam GAMEOVER2.HAS_SPRITE = 1;
	defparam GAMEOVER2.STATIONARY = 1;
	defparam GAMEOVER2.INIT_FILE = "./MIF/gameover2.mif";
	defparam GAMEOVER2.X_INIT = 9'd85;
	defparam GAMEOVER2.Y_INIT = 8'd83;
	defparam GAMEOVER2.KK = 19;

	// Game Over 3: Score 15-19
	object GAMEOVER3 (
		.Resetn(Resetn),
		.Clock(CLOCK_50),
		.gnt(gnt_gameover3),
		.sel(1'b0),
		.jump_trigger(1'b0),
		.duck_trigger(1'b0),
		.new_color(9'b000000000),
		.faster(1'b0),
		.slower(1'b0),
		.speed_level(8'd0),
		.req(req_gameover3),
		.VGA_x(gameover3_x),
		.VGA_y(gameover3_y),
		.VGA_color(gameover3_color),
		.VGA_write(gameover3_write),
		.BASE_X(),
		.BASE_Y()
	);
	defparam GAMEOVER3.nX = nX;
	defparam GAMEOVER3.nY = nY;
	defparam GAMEOVER3.XSCREEN = 320;
	defparam GAMEOVER3.YSCREEN = 240;
	defparam GAMEOVER3.MODE = 2;
	defparam GAMEOVER3.xOBJ = 8;  // 2^8 = 256
	defparam GAMEOVER3.yOBJ = 7;  // 2^7 = 128
	defparam GAMEOVER3.GAMEOVER_WIDTH = 150;
	defparam GAMEOVER3.GAMEOVER_HEIGHT = 75;
	defparam GAMEOVER3.HAS_SPRITE = 1;
	defparam GAMEOVER3.STATIONARY = 1;
	defparam GAMEOVER3.INIT_FILE = "./MIF/gameover3.mif";
	defparam GAMEOVER3.X_INIT = 9'd85;
	defparam GAMEOVER3.Y_INIT = 8'd83;
	defparam GAMEOVER3.KK = 19;

	// Game Over 4: Score >= 20
	object GAMEOVER4 (
		.Resetn(Resetn),
		.Clock(CLOCK_50),
		.gnt(gnt_gameover4),
		.sel(1'b0),
		.jump_trigger(1'b0),
		.duck_trigger(1'b0),
		.new_color(9'b000000000),
		.faster(1'b0),
		.slower(1'b0),
		.speed_level(8'd0),
		.req(req_gameover4),
		.VGA_x(gameover4_x),
		.VGA_y(gameover4_y),
		.VGA_color(gameover4_color),
		.VGA_write(gameover4_write),
		.BASE_X(),
		.BASE_Y()
	);
	defparam GAMEOVER4.nX = nX;
	defparam GAMEOVER4.nY = nY;
	defparam GAMEOVER4.XSCREEN = 320;
	defparam GAMEOVER4.YSCREEN = 240;
	defparam GAMEOVER4.MODE = 2;
	defparam GAMEOVER4.xOBJ = 8;  // 2^8 = 256
	defparam GAMEOVER4.yOBJ = 7;  // 2^7 = 128
	defparam GAMEOVER4.GAMEOVER_WIDTH = 150;
	defparam GAMEOVER4.GAMEOVER_HEIGHT = 75;
	defparam GAMEOVER4.HAS_SPRITE = 1;
	defparam GAMEOVER4.STATIONARY = 1;
	defparam GAMEOVER4.INIT_FILE = "./MIF/gameover4.mif";
	defparam GAMEOVER4.X_INIT = 9'd85;
	defparam GAMEOVER4.Y_INIT = 8'd83;
	defparam GAMEOVER4.KK = 19;

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
	defparam VGA.RESOLUTION="320x240";

	defparam VGA.BACKGROUND_IMAGE ="./MIF/bmp_320_9.mif";

	// --- DINO HITBOX (20x30 inside a 32x32 image) ---
    localparam HB_W = 9'd20;         
    localparam HB_H = 8'd30;         
    localparam HB_X_OFS = 9'd6;      
    localparam HB_Y_OFS = 8'd2;      

    // --- OBSTACLE HITBOX (16x16 inside a 16x16 image) ---
    localparam HB_W_OBS = 9'd16;     
    localparam HB_H_OBS = 8'd16;     
    localparam HB_X_OFS_OBS = 9'd0;  
    localparam HB_Y_OFS_OBS = 8'd0;

	wire [7:0] current_dino_h;
    assign current_dino_h = (duck_combined) ? 8'd15 : 8'd30;

    // 2. Collision Math
    // We use current_dino_h instead of the static HB_H for the dinosaur
    assign collision = (
        (dino_base_x + HB_X_OFS + HB_W  >= obstacle_base_x + HB_X_OFS_OBS) &&
        (dino_base_x + HB_X_OFS         <= obstacle_base_x + HB_X_OFS_OBS + HB_W_OBS) &&
        (dino_base_y + HB_Y_OFS + current_dino_h >= obstacle_base_y + HB_Y_OFS_OBS) &&
        (dino_base_y + HB_Y_OFS <= obstacle_base_y + HB_Y_OFS_OBS + HB_H_OBS)
    );



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
		.high_score(high_score),
		.respawn_obstacle(respawn_obstacle),
		.speed_level(speed_level)
	);
	defparam SCORE.nX = nX;  // Pass updated nX parameter

	wire [3:0] ones, tens, hundreds;
	wire [3:0] high_ones, high_tens, high_hundreds;

	assign ones = score % 10;
	assign tens = (score / 10) % 10;
	assign hundreds = (score / 100) % 10;

	assign high_ones = high_score % 10;
	assign high_tens = (high_score/10) % 10;
	assign high_hundreds = (high_score/100)%10;

	hex_display H0 (ones, HEX0);
	hex_display H1 (tens, HEX1);
	hex_display H2 (hundreds, HEX2);
	hex_display H3(high_ones, HEX3);
	hex_display H4(high_tens, HEX4);
	hex_display H5(high_hundreds, HEX5);

	assign LEDR[0] = collision_latched;

endmodule

// Collision Latch Module
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

// Enhanced Score Counter Module - UPDATED FOR 320x240
module score_counter(Clock, Resetn, dino_x, obstacle_x, collision,
	collision_latched, score, high_score, respawn_obstacle, speed_level);
	parameter nX = 9;  // Changed from 10

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

	initial begin
		high_score = 16'd0;
	end

	always @(posedge Clock) begin
		if (!Resetn) begin
			prev_obstacle_x <= 9'd320;  // Changed from 10'd640 to 9'd320
			obstacle_passed <= 1'b0;
			score_given <= 1'b0;
			prev_collision <= 1'b0;
			prev_collision_latched <= 1'b0;
			score <= 16'd0;
			respawn_obstacle <= 1'b0;
			speed_level <= 8'd0;
		end
		else begin
			prev_obstacle_x <= obstacle_x;
			prev_collision <= collision;
			prev_collision_latched <= collision_latched;
			respawn_obstacle <= 1'b0;

			if (collision_latched && !prev_collision_latched) begin
				if (score > high_score) begin
					high_score <= score;
				end
			end

			if (!collision_latched && prev_collision_latched) begin
				score <= 16'd0;
				speed_level <= 8'd0;
			end

			if (!collision_latched) begin
				if (prev_obstacle_x > dino_x && obstacle_x <= dino_x && !score_given) begin
					if (!collision && !prev_collision) begin
						score <= score + 16'd1;
						speed_level <= speed_level + 8'd1;
						respawn_obstacle <= 1'b1;
					end
					score_given <= 1'b1;
				end

				// Changed threshold from 10'd500 to 9'd250 (halved)
				if (obstacle_x > 9'd250 && prev_obstacle_x < 9'd50) begin
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
			4'h0: display = 7'b1000000;
			4'h1: display = 7'b1111001;
			4'h2: display = 7'b0100100;
			4'h3: display = 7'b0110000;
			4'h4: display = 7'b0011001;
			4'h5: display = 7'b0010010;
			4'h6: display = 7'b0000010;
			4'h7: display = 7'b1111000;
			4'h8: display = 7'b0000000;
			4'h9: display = 7'b0010000;
			default: display = 7'b1111111;
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

// Universal object module - UPDATED FOR 320x240
module object (
    input Resetn, Clock, gnt, sel, 
    input jump_trigger, duck_trigger,
    input faster, slower,
    input [8:0] new_color,
    input [7:0] speed_level,

    output reg req,
    output [nX-1:0] VGA_x,      // Uses parameter nX
    output [nY-1:0] VGA_y,      // Uses parameter nY
    output [8:0] VGA_color,
    output VGA_write,
    
    output [nX-1:0] BASE_X,     // Changed to use nX
    output [nY-1:0] BASE_Y      // Changed to use nY
);


	parameter KK = 19;
	// GRAVITY PARAMETERS
	localparam signed [9:0] GRAVITY = 10'sd1; // pulls down by 1 pixel per tick
	localparam signed [9:0] JUMP_FORCE = -10'sd10; //pushes up by 12 pixels instantly
	localparam signed [9:0] GROUND_Y = 10'd109; // matches Y_INIT

	reg signed [9:0] velocity_y;

    // Parameters - UPDATED FOR 320x240
    parameter nX = 9;            // Changed from 10
    parameter nY = 8;            // Changed from 9
    parameter XSCREEN = 320;     // Changed from 640
    parameter YSCREEN = 240;     // Changed from 480
    parameter MODE = 0;
    parameter STATIONARY = 0;    // New parameter: 1 = object doesn't move, 0 = normal movement
    parameter GAMEOVER_WIDTH = 128;   // Actual gameover image width (for MODE 2)
    parameter GAMEOVER_HEIGHT = 64;   // Actual gameover image height (for MODE 2)

    parameter xOBJ = 5;
    parameter yOBJ = 5;
    parameter BOX_SIZE_X = 1 << xOBJ;
    parameter BOX_SIZE_Y = 1 << yOBJ;
    parameter HAS_SPRITE = 0;
    parameter INIT_FILE = ""; 

    // UPDATED DEFAULT POSITIONS FOR 320x240
    parameter X_INIT = 9'd0;     // Changed from 10'd0
    parameter Y_INIT = 8'd119;   // Changed from 9'd239 (approximately halved)
    parameter JUMP_HEIGHT = 8'd65; // Changed from 9'd80 (halved)
    

    parameter HITBOX_W = 9'd30;      // Changed from 10'd30
    parameter HITBOX_H = 8'd30;      // Changed from 9'd30
    parameter HITBOX_X_OFS = 9'd0;   // Changed from 10'd0
    parameter HITBOX_Y_OFS = 8'd0;   // Changed from 9'd0

    // Internal registers with updated bit widths
    reg [nX-1:0] X_reg, X_prev;
    reg [nY-1:0] Y_reg, Y_prev;
    reg prev_select;

    wire [xOBJ-1:0] XC;
    wire [yOBJ-1:0] YC;

    reg [3:0] draw_Q, draw_D;
    reg [1:0] Jump_Q;
	//reg [1:0] Jump_D;
    parameter Running = 2'b00, Ascending = 2'b01, Descending = 2'b10;

    reg is_ducking;
    reg [25:0] duck_timer;
	localparam DUCK_DURATION = 26'd50_000_000;


    wire [KK-1:0] slow; 
    wire sync_adjusted;

    // Sprite logic
    wire [14:0] sprite_addr;  // 15 bits to support up to 32K addresses
    wire [12:0] sprite_addr_linear;  // For linear addressing (MODE 2) - 13 bits for 8192
    
    // For MODE 2 (gameover): Linear addressing for 128x64 stored in 8192 addresses
    // Address = row * GAMEOVER_WIDTH + col (only valid if within bounds)
    // Valid addresses: 0 to 8,191 (128*64-1)
    // No padding needed - 128×64 = 8192 is already power of 2!
    assign sprite_addr_linear = (YC * GAMEOVER_WIDTH) + XC;
    
    // For MODE 0/1: Standard 2D addressing {YC, XC}
    // For MODE 2: Linear addressing
    assign sprite_addr = (MODE == 2) ? {2'b0, sprite_addr_linear} : {YC, XC};

    wire [8:0] pixel_data_run1, pixel_data_run2, pixel_data_run3, pixel_data_run4;
    wire [8:0] pixel_data_jump, pixel_data_duck;
    reg [8:0] anim_sprite_color; 
    reg [5:0] anim_tick;
    reg [1:0] run_frame;

    wire [8:0] static_sprite_color; 

    wire [5:0] anim_threshold;
    assign anim_threshold = (speed_level > 15) ? 6'd8 : (20 - speed_level);

	wire signed [10:0] current_y_signed = $signed({1'b0, Y_reg});
	wire signed [10:0] next_y_signed = current_y_signed + velocity_y;
	


    generate
        if (HAS_SPRITE && MODE == 1) begin : GEN_PLAYER_SPRITES
            sprite_rom #(.MEM_INIT_FILE("running1.mif")) r1 (.clk(Clock), .addr(sprite_addr), .q(pixel_data_run1));
            sprite_rom #(.MEM_INIT_FILE("running2.mif")) r2 (.clk(Clock), .addr(sprite_addr), .q(pixel_data_run2));
            sprite_rom #(.MEM_INIT_FILE("running3.mif")) r3 (.clk(Clock), .addr(sprite_addr), .q(pixel_data_run3));
            sprite_rom #(.MEM_INIT_FILE("running4.mif")) r4 (.clk(Clock), .addr(sprite_addr), .q(pixel_data_run4));
            sprite_rom #(.MEM_INIT_FILE("jump.mif")) rj (.clk(Clock), .addr(sprite_addr), .q(pixel_data_jump));
            sprite_rom #(.MEM_INIT_FILE("new_duck.mif")) rd (.clk(Clock), .addr(sprite_addr), .q(pixel_data_duck));
        end
    endgenerate

    generate
        if (HAS_SPRITE && MODE == 0) begin : GEN_OBSTACLE_SPRITE
            object_mem #(
                .INIT_FILE(INIT_FILE),
                .n(9),
                .Mn(xOBJ + yOBJ)  // Address width = xOBJ + yOBJ bits
            ) SPRITE_ROM (
                .clock(Clock),
                .address(sprite_addr[xOBJ + yOBJ - 1:0]), 
                .q(static_sprite_color)
            );
        end else if (HAS_SPRITE && MODE == 2) begin : GEN_GAMEOVER_SPRITE
            object_mem #(
                .INIT_FILE(INIT_FILE),
                .n(9),
                .Mn(13)  // 13-bit address for 8192 addresses (2^13 = 128*64)
            ) GAMEOVER_ROM (
                .clock(Clock),
                .address(sprite_addr[12:0]),  // Use lower 13 bits for 8192 addresses
                .q(static_sprite_color)
            );
        end else begin
            assign static_sprite_color = 9'd0;
        end
    endgenerate

    reg [8:0] final_pixel_out;

    always @(*) begin
        if (Jump_Q != Running) begin
            anim_sprite_color = pixel_data_jump;
        end else if (is_ducking) begin
            anim_sprite_color = pixel_data_duck;
        end else begin
            case (run_frame)
                2'd0: anim_sprite_color = pixel_data_run1;
                2'd1: anim_sprite_color = pixel_data_run2;
                2'd2: anim_sprite_color = pixel_data_run3;
                2'd3: anim_sprite_color = pixel_data_run4;
            endcase
        end
    end

    always @(*) begin
        if (!HAS_SPRITE) begin
            final_pixel_out = new_color;
        end else begin
            if (MODE == 1) begin
                final_pixel_out = anim_sprite_color;
            end else begin  // MODE 0 or MODE 2
                final_pixel_out = static_sprite_color;
            end
        end
    end

    upDn_count U3 ({xOBJ{1'd0}}, Clock, Resetn, Lxc, Exc, 1'b1, XC);
        defparam U3.n = xOBJ;
    upDn_count U4 ({yOBJ{1'd0}}, Clock, Resetn, Lyc, Eyc, 1'b1, YC);
        defparam U4.n = yOBJ;

    Up_count U6 (Clock, Resetn, slow);
        defparam U6.n = KK; 

    assign sync_adjusted = (slow == 0);

    wire [15:0] lfsr_out;
    reg [nX-1:0] random_x_offset;  // Changed from [9:0] to [nX-1:0]
    lfsr_16bit RAND_GEN (Clock, Resetn, lfsr_out);

    always @(posedge Clock) begin
        if (!Resetn) random_x_offset <= 0;
        else if (faster && MODE == 0) begin 
             random_x_offset <= (lfsr_out[8:0] % 9'd100);  // Changed from [9:0] and 10'd200
        end
    end

    always @(posedge Clock) begin
        if (!Resetn) begin
            Jump_Q <= Running;
            is_ducking <= 0;
            duck_timer <= 0;
            Y_reg <= Y_INIT;
            velocity_y <= 0;     // Reset velocity
            anim_tick <= 0;
            run_frame <= 0;
        end else begin
            
            // --- DUCK LOGIC (Kept same) ---
            if (duck_trigger && Jump_Q == Running && duck_timer == 0) begin
                is_ducking <= 1;
                duck_timer <= DUCK_DURATION;
            end else if (duck_timer > 0) begin
                duck_timer <= duck_timer - 1;
                is_ducking <= 1; 
                run_frame <= 0;
                anim_tick <= 0;
            end else begin
                is_ducking <= 0;
            end

            // --- ANIMATION TICKS (Kept same) ---
            if (sync_adjusted) begin
                 if (anim_tick >= anim_threshold) begin
                     anim_tick <= 0;
                     run_frame <= run_frame + 1;
                 end else begin
                     anim_tick <= anim_tick + 1;
                 end
            end

            // 1. JUMP TRIGGER (Instant Upward Force)

            // We can only jump if we are currently Running (on the ground)
            if (jump_trigger && !is_ducking && Jump_Q == Running) begin
                velocity_y <= JUMP_FORCE;
                Jump_Q <= Ascending; // Set state to "In Air"
            end
            
            // Apply Gravity & Movement
            // Only update physics on the sync_adjusted tick to control speed
            else if (MODE == 1 && sync_adjusted) begin
                // If we are in the air...
                if (Jump_Q != Running) begin
                    
                    // Update Velocity (Gravity pulls down)
                    velocity_y <= velocity_y + GRAVITY;
                    
                    // PREDICT LANDING: Check if next move hits ground
                    if (next_y_signed >= $signed({1'b0, GROUND_Y})) begin
    					Y_reg <= GROUND_Y;      // Snap to ground
						velocity_y <= 0;        // Stop falling
						Jump_Q <= Running;      // State: Running
					end else begin
						Y_reg <= next_y_signed[nY-1:0]; // Move dino
					end
                end 
                // Safety: Force ground height if Running
                else begin
                    Y_reg <= GROUND_Y;
                    velocity_y <= 0;
                end
            end
        end
    end


    always @(posedge Clock) begin
        if (!Resetn) X_reg <= X_INIT;
        else if (MODE == 0 && !STATIONARY) begin 
           if (faster) begin
               X_reg <= XSCREEN - BOX_SIZE_X + random_x_offset;
           end else if (sync_adjusted) begin
               if (X_reg <= 1) X_reg <= XSCREEN - BOX_SIZE_X;
               else X_reg <= X_reg - 1'b1;
           end
        end
        // If STATIONARY == 1 or MODE == 2, X_reg stays at X_INIT
    end

    // --- IMPROVED GHOST CLEANUP LOGIC ---
    reg ignore_first_tick;

    // 1. Control the "Ignore" flag
    always @(posedge Clock) begin
        if (!Resetn) begin
            // On Reset, raise the flag: "Don't update history yet!"
            ignore_first_tick <= 1'b1;
        end
        else if (sync_adjusted) begin
            // On the first tick, this will still be 1. 
            // We clear it so the NEXT tick works normally.
            ignore_first_tick <= 1'b0;
        end
    end

    // 2. Smart History Update
    always @(posedge Clock) begin
        if (!Resetn) begin
            // Reset held: Keep X_prev at the "Crash Site"
            // X_reg is resetting to "Start Site" in its own block
        end
        else if (sync_adjusted) begin
            // Only update X_prev if we are NOT in the first tick after reset
            if (!ignore_first_tick) begin
                X_prev <= X_reg;
                Y_prev <= Y_reg;
            end
        end
    end

    parameter D_A=0, D_B=1, D_C=2, D_D=3, D_E=4, D_F=5, D_G=6, D_H=7, D_I=8, D_J=9, D_K=10, D_L=11;
    reg Lx, Ly, Lxc, Lyc, Exc, Eyc, erase, write;

    always @(posedge Clock) begin
        if (!Resetn) draw_Q <= D_A;
        else draw_Q <= draw_D;
    end

    always @(*) case (draw_Q)
        D_A: draw_D = D_B;
        D_B: if (XC != BOX_SIZE_X-1) draw_D = D_B; else draw_D = D_C;
        D_C: if (YC != BOX_SIZE_Y-1) draw_D = D_B; else draw_D = D_D;
        D_D: if (!sync_adjusted) draw_D = D_D; else draw_D = D_E;
        D_E: if (!gnt) draw_D = D_E; else draw_D = D_F;
        D_F: if (XC != BOX_SIZE_X-1) draw_D = D_F; else draw_D = D_G;
        D_G: if (YC != BOX_SIZE_Y-1) draw_D = D_F; else draw_D = D_H;
        D_H: draw_D = D_I;
        D_I: draw_D = D_J;
        D_J: if (XC != BOX_SIZE_X-1) draw_D = D_J; else draw_D = D_K;
        D_K: if (YC != BOX_SIZE_Y-1) draw_D = D_J; else draw_D = D_L;
        D_L: draw_D = D_D;
        default: draw_D = D_A;
    endcase

    always @(*) begin
        Lx = 0; Ly = 0; Lxc = 0; Lyc = 0; Exc = 0; Eyc = 0;
        erase = 0; write = 0; req = 0; prev_select = 0;

        case (draw_Q)
            D_A: begin Lx=1; Ly=1; Lxc=1; Lyc=1; end
            D_B: begin Exc=1; write=1; end
            D_C: begin Lxc=1; Eyc=1; end
            D_D: Lyc=1;
            D_E: req=1;
            D_F: begin req=1; Exc=1; erase=1; write=1; prev_select=1; end
            D_G: begin req=1; Lxc=1; Eyc=1; prev_select=1; end
            D_H: begin req=1; Lyc=1; end
            D_I: begin req=1; end
            D_J: begin req=1; Exc=1; write=1; end
            D_K: begin req=1; Lxc=1; Eyc=1; end
            D_L: Lyc=1;
		endcase
    end
    
    localparam SPRITE_Y_OFFSET = 10; 
	localparam DUCK_Y_SHIFT = 0;

    assign VGA_x = ((prev_select) ? X_prev : X_reg) + XC;
    assign VGA_y = ((prev_select) ? Y_prev : Y_reg) + YC 
                 + ((MODE==1)? SPRITE_Y_OFFSET : 0)  // Only for player (MODE 1)
                 + ((is_ducking) ? DUCK_Y_SHIFT : 0);
    
    assign VGA_color = (erase) ? 9'b111111111 : final_pixel_out;
    // --- DYNAMIC TRANSPARENCY LOGIC ---
    
    wire [8:0] transparent_color;
    
    // Transparency Logic: 
    // MODE 1 (Dino) + Ducking: transparent = BLACK (0) - for ducking sprite
    // MODE 2 (Gameover): NO TRANSPARENCY - all pixels drawn
    // MODE 0 (Obstacle) or MODE 1 (Dino) not ducking: transparent = WHITE (all 1s)
    assign transparent_color = (MODE == 1 && is_ducking) ? 9'd0 : 9'b111111111;

    // For MODE 2, only write pixels within actual image bounds (128x64)
    wire within_gameover_bounds;
    assign within_gameover_bounds = (MODE == 2) ? 
                                   (XC < GAMEOVER_WIDTH && YC < GAMEOVER_HEIGHT) : 
                                   1'b1;  // Always true for other modes

    // VGA_write logic:
    // MODE 2: Write ALL pixels within bounds (no transparency check)
    // Other modes: Use transparency check
    assign VGA_write = (MODE == 2) ? 
                      (write & within_gameover_bounds) :
                      (write & (erase || (final_pixel_out != transparent_color)));

    assign BASE_X = X_reg;
    assign BASE_Y = Y_reg;



endmodule


// 16-bit LFSR
module lfsr_16bit(Clock, Resetn, random_out);
	input Clock, Resetn;
	output reg [15:0] random_out;

	wire feedback;
	assign feedback = random_out[15] ^ random_out[14] ^ random_out[12] ^ random_out[3];

	always @(posedge Clock) begin
		if (!Resetn)
			random_out <= 16'hACE1;
		else
			random_out <= {random_out[14:0], feedback};
	end
endmodule

// Baud rate generator
module baud16x #(parameter CLK_HZ=50_000_000, BAUD=115200)(
	input wire clk,
	input wire rst_n,
	output reg tick_16x
);
	localparam integer DIV = CLK_HZ/(BAUD*16);
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

// UART RX with 16× oversampling
module uart_rx_16x #(
	parameter CLK_HZ=50_000_000,
	parameter BAUD =115200
)(
	input wire clk,
	input wire rst_n,
	input wire os_tick,
	input wire rx_raw,
	output reg data_ready,
	output reg [7:0] data
);
	reg rx_m, rx_s;
	always @(posedge clk) begin
		rx_m <= rx_raw;
		rx_s <= rx_m;
	end

	localparam IDLE=2'd0, START=2'd1, DATA=2'd2, STOP=2'd3;
	reg [1:0] st;
	reg [3:0] os_cnt;
	reg [2:0] bit_i;
	reg [7:0] sh;

	always @(posedge clk) begin
		if (!rst_n) begin
			st <= IDLE;
			os_cnt <= 4'd0;
			bit_i <= 3'd0;
			sh <= 8'h00;
			data <= 8'h00;
			data_ready <= 1'b0;
		end else begin
			data_ready <= 1'b0;

			if (os_tick) begin
				case (st)
					IDLE: if (rx_s==1'b0) begin st<=START; os_cnt<=4'd7; end
					START: if (os_cnt==0) begin
						if (rx_s==1'b0) begin st<=DATA; os_cnt<=4'd15; bit_i<=3'd0; end
						else st<=IDLE;
					end else os_cnt<=os_cnt-1'b1;
					DATA: if (os_cnt==0) begin
						sh[bit_i] <= rx_s;
						os_cnt <= 4'd15;
						if (bit_i==3'd7) st<=STOP; else bit_i<=bit_i+1'b1;
					end else os_cnt<=os_cnt-1'b1;
					STOP: if (os_cnt==0) begin
						data <= sh;
						data_ready <= 1'b1;
						st <= IDLE;
					end else os_cnt<=os_cnt-1'b1;
				endcase
			end
		end
	end
endmodule

// UART command decoder
module uart_cmd_decoder(
	input wire clk,
	input wire rst_n,
	input wire rx_strobe,
	input wire [7:0] rx_data,
	output reg jump_pulse,
	output reg duck_pulse,
	output reg idle_pulse
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
					"J": jump_pulse <= 1'b1;
					"D": duck_pulse <= 1'b1;
					"I": idle_pulse <= 1'b1;
					default: ;
				endcase
			end
		end
	end
endmodule

// Object memory module
module object_mem (address, clock, q);
	parameter n = 3;
	parameter Mn = 6;
	parameter INIT_FILE = "./MIF/object_mem_8_8_3.mif";

	input wire [Mn-1:0] address;
	input wire clock;
	output [n-1:0] q;
	wire [n-1:0] sub_wire0;
	wire [n-1:0] q = sub_wire0[n-1:0];

	altsyncram altsyncram_component (
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

// Sprite ROM module
module sprite_rom #(
    parameter MEM_INIT_FILE = "",
    parameter DATA_WIDTH = 9,
    parameter ADDR_WIDTH = 10  
)(
    input clk,
    input [ADDR_WIDTH-1:0] addr,
    output reg [DATA_WIDTH-1:0] q
);
    (* ram_init_file = MEM_INIT_FILE *) reg [DATA_WIDTH-1:0] rom [0:(1<<ADDR_WIDTH)-1];
    
    always @(posedge clk) begin
        q <= rom[addr];
    end
endmodule