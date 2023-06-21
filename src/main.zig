const std = @import("std");
const Alloc = std.mem.Allocator;
const Vec = std.ArrayList;
const c = @cImport(
    @cInclude("raylib.h"),
);

pub const Elevator = struct {
    const BitSetT = std.bit_set.DynamicBitSet;

    const max_v: f32 = 0.3;
    const accel: f32 = 0.2;
    const brake: f32 = 0.4;

    velocity: f32,
    position: f32,
    parked: bool = true,
    target_floor: ?usize = null,

    phys_state: PhysicsState = .stopped,

    cab_calls: BitSetT,
    hall_calls_up: BitSetT,
    hall_calls_down: BitSetT,

    floor_map: Vec(FloorMapItem),

    pub const FloorMapItem = struct { rel_y: f32 = 1 };

    pub fn init(alloc: Alloc, floor_map: []const FloorMapItem) !@This() {
        var fmap = Vec(FloorMapItem).init(alloc);
        try fmap.appendSlice(floor_map);
        const t = 0;
        return .{
            .floor_map = fmap,
            .velocity = 0,
            .position = 4,
            .cab_calls = try BitSetT.initEmpty(alloc, t),
            .hall_calls_up = try BitSetT.initEmpty(alloc, t),
            .hall_calls_down = try BitSetT.initEmpty(alloc, t),
        };
    }

    pub const PhysicsState = enum { accel, brake, constant, stopped };

    pub fn deinit(self: *@This()) void {
        self.cab_calls.deinit();
        self.hall_calls_up.deinit();
        self.hall_calls_down.deinit();
    }

    pub fn getAbsoluteFloorPosition(floors: []const FloorMapItem, floor_i: usize) f32 {
        var pos: f32 = 0;
        var i: usize = 0;
        while (i < floor_i) : (i += 1) {
            pos += floors[i].rel_y;
        }
        return pos;
    }

    pub fn update_phys(self: *@This(), dt: f32, dir: ?enum { up, down }) void {
        //std.debug.print("V: {d}, {any}, dt: {d}, dir: {any}, pos: {d}\n", .{ self.velocity, self.phys_state, dt, dir, self.position });
        var sf: f32 = if (self.velocity < 0) -1.0 else 1.0;
        switch (self.phys_state) {
            .accel => {
                if (self.velocity == 0) {
                    const d = dir orelse unreachable;
                    sf = if (d == .up) -1.0 else 1.0;
                }
                const absv = @fabs(self.velocity);
                if (absv > max_v) unreachable;
                const max_dt = (max_v - absv) / accel;

                if (max_dt <= dt) {
                    self.velocity = max_v * sf;
                    self.position += sf * 0.5 * accel * (max_dt * max_dt) + (self.velocity * max_dt);

                    self.phys_state = .constant;
                    self.update_phys(dt - max_dt, null);
                    return;
                } else {
                    self.position += (sf * 0.5 * accel * (dt * dt)) + (self.velocity * dt);
                    self.velocity += sf * dt * accel;
                }
            },
            .brake => {
                const absv = @fabs(self.velocity);
                if (absv > max_v) unreachable;
                const max_dt = (absv) / brake;

                if (max_dt <= dt) {
                    self.position += sf * -0.5 * brake * (max_dt * max_dt) + (self.velocity * max_dt);
                    self.velocity = 0;
                    self.phys_state = .stopped;
                    self.update_phys(dt - max_dt, null);
                } else {
                    self.position += sf * -0.5 * brake * (dt * dt) + (self.velocity * dt);
                    self.velocity += sf * dt * (-brake);
                }
            },
            .constant => {
                const target_pos = blk: {
                    if (self.target_floor) |floor_i| {
                        break :blk getAbsoluteFloorPosition(self.floor_map.items, floor_i);
                    }
                    break :blk 0; //got to our "home" floor;
                };
                const target_dir: f32 = if (target_pos - self.position < 0) -1.0 else 1.0;

                //Are we heading towards the target, no then brake;
                if (target_dir * sf < 0) {
                    self.phys_state = .brake;
                    self.update_phys(dt, null);
                    return;
                }

                //will we overshoot the brake distance in this dt, yes then apply our vel and
                {
                    const t = (max_v / brake);
                    const brake_dist = (0.5 * (-brake) * (t * t)) + (max_v * t);
                    const dxt = @fabs(target_pos - self.position);
                    const dist_before_brake = dxt - brake_dist;

                    const maxdt = (dist_before_brake / max_v);
                    if (dt > maxdt) {
                        self.position += self.velocity * maxdt;
                        self.phys_state = .brake;
                        self.update_phys(dt - maxdt, null);
                        return;
                    }
                }

                self.position += self.velocity * dt;
            },
            .stopped => {
                if (self.target_floor) |target_floor| {
                    const absfloor = getAbsoluteFloorPosition(self.floor_map.items, target_floor);
                    if (@fabs(absfloor - self.position) < 0.01) {
                        return;
                    }
                    self.phys_state = .accel;
                    self.update_phys(dt, if (absfloor > self.position) .down else .up);
                }
            },
        }
    }

    pub fn update(self: *@This(), dt: f32) void {
        if (self.target_floor) |floor_i| {
            const target_pos = getAbsoluteFloorPosition(self.floor_map.items, floor_i);
            _ = target_pos;
            _ = dt;
        }
    }
};

pub fn draw_hall_call_indicator(pos: c.Vector2, up: bool, down: bool) void {
    c.DrawTriangle(pos, .{ .x = pos.x - 10, .y = pos.y - 20 }, .{ .x = pos.x - 20, .y = pos.y }, if (up) c.ORANGE else c.WHITE);
    c.DrawTriangle(.{ .x = pos.x, .y = pos.y + 5 }, .{ .x = pos.x - 20, .y = pos.y + 5 }, .{ .x = pos.x - 10, .y = pos.y + 5 + 20 }, if (down) c.ORANGE else c.WHITE);
}

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    const alloc = gpa.allocator();

    var elevator = try Elevator.init(alloc, &[_]Elevator.FloorMapItem{ .{}, .{ .rel_y = 0.5 }, .{ .rel_y = 2 }, .{}, .{} });
    defer elevator.deinit();

    c.InitWindow(800, 600, "test");
    c.SetTargetFPS(60);

    while (!c.WindowShouldClose()) {
        elevator.update_phys(0.16, null);
        {
            if (c.IsKeyPressed(c.KEY_A)) {
                elevator.phys_state = .accel;
            }
            if (c.IsKeyPressed(c.KEY_B)) {
                elevator.phys_state = .brake;
            }

            var k = c.GetKeyPressed();
            while (k != 0) : (k = c.GetKeyPressed()) {
                switch (k) {
                    c.KEY_ZERO...c.KEY_NINE => {
                        const floor_num = k - c.KEY_ZERO;
                        elevator.target_floor = @intCast(usize, floor_num);
                    },
                    else => {},
                }
            }
        }

        c.BeginDrawing();
        c.ClearBackground(c.GRAY);
        {
            var y: f32 = 0;
            const sfy = 100;
            for (elevator.floor_map.items) |floor, i| {
                const pos = c.Vector2{ .x = 40, .y = floor.rel_y * sfy };
                draw_hall_call_indicator(.{ .x = pos.x, .y = y + (floor.rel_y / 2) * sfy }, false, false);
                const color = if (i % 2 == 0) c.WHITE else c.BLACK;
                c.DrawRectangleV(.{ .x = sfy, .y = y }, pos, color);
                y += floor.rel_y * sfy;
            }

            c.DrawRectangleV(.{ .x = sfy, .y = elevator.position * sfy }, .{ .x = 20, .y = sfy / 2 }, c.BLUE);
        }
        {
            var _buf: [256]u8 = undefined;
            var fbs = std.io.FixedBufferStream([]u8){ .buffer = &_buf, .pos = 0 };
            try fbs.writer().print("{any}", .{elevator.phys_state});
            try fbs.writer().writeByte(0);
            c.DrawText(@ptrCast([*c]u8, fbs.getWritten()), 60, 40, 20, c.BLACK);
        }
        c.EndDrawing();
    }
}
