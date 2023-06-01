module RGB_to_HSV (R,G,B,H,S,V);

input wire [7:0] R;
input wire [7:0] G;
input wire [7:0] B;
output wire [7:0] H;
output wire [7:0] S;
output wire [7:0] V;

wire [1:0] fmax;
wire [7:0] cmax;
wire [7:0] cmin;
wire [7:0] C;
wire [15:0] t0;
wire [15:0] t1;

assign cmin = (R < G & R < B) ? R : (G < B) ? G : B;
assign cmax = (R > G & R > B) ? R : (G > B) ? G : B;
assign C = cmax - cmin;

assign H = (cmax == 0 | C == 0) ? 8'b0 : (cmax == R) ? (43*(G-B)/C) : (cmax == G) ? ((43*(B-R)/C)+85): ((43*(R-G)/C)+171);

assign S  = (cmax == 0 | C == 0) ? 8'b0 : ((255*C)/cmax);

assign V = cmax;

endmodule
