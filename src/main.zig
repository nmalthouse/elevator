const std = @import("std");
const Alloc = std.mem.Allocator;
const Vec = std.ArrayList;
const graph = @import("graph");
const Dctx = graph.ImmediateDrawingContext;
const Rect = graph.Rect;
const Rec = graph.Rec;
const Vec2f = graph.Vec2f;
const Colors = graph.Colori;

pub fn pow(x: f32, y: f32) f32 {
    return std.math.pow(f32, x, y);
}

pub const Agent = struct {
    const Self = @This();
    const elevator_x: f32 = 5;
    const speed: f32 = 3;
    x: f32,
    floor_i: usize,
    state: State,

    goal_x: f32,
    goal_floor: usize,

    attached_to_elevator: bool = false,

    pub const State = enum {
        walking,
        standing,
        elevating,
    };

    pub fn isOnGoalFloor(self: *const Self) bool {
        return (self.floor_i == self.goal_floor);
    }

    pub fn isAtGoalX(self: *const Self) bool {
        return (@fabs(self.goal_x - self.x) < 0.1);
    }

    pub fn isAtElevatorX(self: *const Self) bool {
        return @fabs(self.x - elevator_x) < 0.1;
    }

    pub fn update(self: *@This(), dt: f32, elevator: *Elevator) void {
        switch (self.state) {
            .standing => {
                if (!self.isOnGoalFloor()) {
                    if (self.isAtElevatorX()) {
                        if (elevator.isParkedAt(self.floor_i)) {
                            //elevator.getOnLol()
                            self.state = .elevating;
                            elevator.cabCall(self.goal_floor);
                        }
                    } else {
                        self.state = .walking;
                    }
                } else {
                    if (!self.isAtGoalX()) {
                        self.state = .walking;
                    } else { //We are home
                    }
                }
            },
            .walking => {
                if (!self.isOnGoalFloor()) {
                    if (!self.isAtElevatorX()) { //Walk to elevator
                        const sf: f32 = if (self.x > elevator_x) -1.0 else 1.0;
                        const max_dt = @fabs((elevator_x - self.x) / speed);
                        self.x += sf * speed;
                        const new_sf: f32 = if (self.x > elevator_x) -1.0 else 1.0;
                        if (new_sf * sf < 0) {
                            //self.state = .standing;
                            self.x = elevator_x;
                            self.update(dt - max_dt, elevator);
                        }
                    } else {
                        elevator.hallCall(
                            self.floor_i,
                            if (self.goal_floor > self.floor_i) .down else .up,
                        );
                        self.state = .standing;
                    }
                } else {
                    if (!self.isAtGoalX()) {
                        const g = self.goal_x;
                        const sf: f32 = if (self.x > g) -1.0 else 1.0;
                        const max_dt = @fabs((g - self.x) / speed);
                        self.x += sf * speed;
                        const new_sf: f32 = if (self.x > g) -1.0 else 1.0;
                        if (new_sf * sf < 0) {
                            //self.state = .standing;
                            self.x = g;
                            self.update(dt - max_dt, elevator);
                        }
                    } else {
                        self.state = .standing;
                    }
                }
            },
            .elevating => {
                if (elevator.isParkedAt(self.goal_floor)) {
                    self.floor_i = self.goal_floor;
                    self.state = .standing;
                }
            },
        }
    }
};

//Rules of the elevator
//A Elevator moves from floor a to b. The speed curve of the elevator has 2 possibilities:
//  curve A: stopped accel, constant, brake, stopped
//  curve B: stopped, accel, brake, stopped
//
//  The curve used is always determined in the stopped state and persists until next stop
//
//The call dispatching algorithm works as follows
//  The next floor can only be changed in stopped or constant state.
//  The elevator will move in one direction until all calls in that direction are exhausted
//  When stopped the elevator first searches for the nearest cab call or hall call with matching direction in the cabs current direction, choosing the nearest one.
//  If no call is found look for the farthest hall_call in the direction opposite to the elevators in the elevators current direction
//  If no call is found reverse the elevator direction and search again
//
//  If the target floor is too near to the cab to accelerate fully, calculate and use velocity curve B.
//  Change state to accel.
//
//  while the state is constant speed
//  check for floors with matching calls that our within our braking distance and change target floor and state if a call is found

//Guarantee position and velocity of cab are set to specific values when stopped i.e. calculate the exact position the cab should be rather than rely on our physics algorithm when transitioning from brake to stopped
const BitSetT = std.bit_set.DynamicBitSet;
pub const Elevator = struct {
    const Self = @This();
    const max_v: f32 = 1.3;
    const accel: f32 = 0.2;
    const brake: f32 = 0.4;

    const leveling_error: f32 = 0.1; //The unit is floors
    const stop_time: f32 = 1.25;

    parked_at: ?usize = null,
    stop_timer: f32 = 0,
    velocity: f32,
    position: f32,
    target_floor: ?usize = null,
    last_direction: Direction = .down,
    speed_limit: f32 = max_v,

    phys_state: PhysicsState = .stopped,

    cab_calls: BitSetT,
    hall_calls_up: BitSetT,
    hall_calls_down: BitSetT,
    num_passengers: u32 = 0,

    floor_map: Vec(FloorMapItem),

    pub const FloorMapItem = struct {
        rel_y: f32 = 1, //Distance from last floor to this floor
    };
    pub const Direction = enum {
        pub fn invert(self: @This()) @This() {
            return switch (self) {
                .up => .down,
                .down => .up,
            };
        }

        up, //Up is toward 0
        down, //Down is away from 0
    };

    pub fn init(alloc: Alloc, floor_map: []const FloorMapItem) !@This() {
        var fmap = Vec(FloorMapItem).init(alloc);
        try fmap.appendSlice(floor_map);
        return .{
            .floor_map = fmap,
            .velocity = 0,
            .position = 0,
            .cab_calls = try BitSetT.initEmpty(alloc, floor_map.len),
            .hall_calls_up = try BitSetT.initEmpty(alloc, floor_map.len),
            .hall_calls_down = try BitSetT.initEmpty(alloc, floor_map.len),
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
        while (i <= floor_i) : (i += 1) {
            pos += floors[i].rel_y;
        }
        return pos;
    }

    pub fn isParkedAt(self: *Self, floor: usize) bool {
        if (self.parked_at) |p| {
            return p == floor;
        }
        return false;
    }

    pub fn cabCall(self: *@This(), floor: usize) void {
        if (self.parked_at != null and floor == self.parked_at.?) return;
        self.cab_calls.set(floor);
    }

    pub fn hallCall(self: *@This(), floor: usize, dir: Direction) void {
        if (self.parked_at != null and floor == self.parked_at.?) return;
        switch (dir) {
            .up => {
                if (floor == 0) return; //We can't go any further up
                self.hall_calls_up.set(floor);
            },
            .down => {
                //if (floor == self.floor_map.len) return;
                self.hall_calls_down.set(floor);
            },
        }
    }

    pub fn getNearestFloor(floors: []const FloorMapItem, pos: f32) usize {
        var i: usize = 0;
        var apos: f32 = 0;
        const lt = blk: {
            if (floors[0].rel_y > pos) return 0;
            while (true) {
                apos += floors[i].rel_y;
                if (apos >= pos) break :blk apos - floors[i].rel_y;
                if (i == floors.len - 1) return floors.len - 1;
                i += 1;
            }
        };
        const gtd = @fabs(apos - pos);
        const ltd = @fabs(lt - pos);
        return if (gtd < ltd) i else i - 1;
    }

    //Get the nearest floor to POS but not at POS in a given direction;
    pub fn getNearestFloorIndexDir(floors: []const FloorMapItem, pos: f32, dir: Direction) ?usize {
        switch (dir) {
            .up => {
                var abs_floor_pos: f32 = getAbsoluteFloorPosition(floors, floors.len - 1);
                var index: usize = floors.len - 1;

                if (pos <= floors[0].rel_y) return null;

                while (true) {
                    if (abs_floor_pos < pos) return index;
                    abs_floor_pos -= floors[index].rel_y;
                    index -= 1;
                }
            },
            .down => {
                var index: usize = 0;
                var abs_floor_pos: f32 = floors[index].rel_y;
                if (pos >= getAbsoluteFloorPosition(floors, floors.len - 1)) return null;

                while (true) {
                    if (abs_floor_pos > pos) return index;
                    index += 1;
                    abs_floor_pos += floors[index].rel_y;
                }
            },
        }
        unreachable;
    }

    pub fn calculate_speed_limit(travel_dist: f32) f32 {
        const accel_time = @sqrt(travel_dist / ((accel / 2) + (pow(accel, 2) / (2 * brake))));
        const apex_v = accel_time * accel;
        return if (apex_v > max_v) max_v else apex_v;
    }

    pub fn findNearestCall(floors: []const FloorMapItem, floor_i: usize, dir: Direction, call_set: *const BitSetT) ?usize {
        var i: usize = floor_i;
        while (true) {
            if (call_set.isSet(i)) return i;
            if (i + 1 <= 0 or i >= floors.len - 1) return null;
            switch (dir) {
                .up => {
                    if (i == 0) return null;
                    i -= 1;
                },
                .down => {
                    if (i == floors.len - 1) return null;
                    i += 1;
                },
            }
        }
    }

    pub fn findFarthestCall(floors: []const FloorMapItem, floor_i: usize, dir: Direction, call_set: *const BitSetT) ?usize {
        var i: usize = switch (dir) {
            .up => 0,
            .down => floors.len - 1,
        };
        while (true) {
            if (call_set.isSet(i)) return i;
            if (i == floor_i) return null;
            switch (dir) {
                .up => {
                    if (i == floors.len - 1) return null;
                    i += 1;
                },
                .down => {
                    if (i == 0) return null;
                    i -= 1;
                },
            }
        }
    }

    pub fn getNextCallInDirection(self: *const @This(), pos: f32, dir: Direction) ?usize {
        const floors = self.floor_map.items;
        if (getNearestFloorIndexDir(floors, pos, dir)) |nearest_floor_i| {
            const hall_call = findNearestCall(floors, nearest_floor_i, dir, switch (dir) {
                .up => &self.hall_calls_up,
                .down => &self.hall_calls_down,
            });
            const cab_call = findNearestCall(floors, nearest_floor_i, dir, &self.cab_calls);

            const next_floor = blk: {
                if (hall_call != null and cab_call != null) {
                    switch (dir) {
                        .up => break :blk if (hall_call.? > cab_call.?) hall_call.? else cab_call.?,
                        .down => break :blk if (hall_call.? > cab_call.?) cab_call.? else hall_call.?,
                    }
                }
                if (hall_call != null) break :blk hall_call.?;
                if (cab_call != null) break :blk cab_call.?;

                if (findFarthestCall(floors, nearest_floor_i, dir, switch (dir) {
                    .up => &self.hall_calls_down,
                    .down => &self.hall_calls_up,
                })) |farthest_call| {
                    break :blk farthest_call;
                }
                break :blk null;
            };
            return next_floor;
        }
        return null;
    }

    pub fn update_phys(self: *@This(), dt: f32) void {
        //std.debug.print("V: {d}, {any}, dt: {d}, dir: {any}, pos: {d}\n", .{ self.velocity, self.phys_state, dt, dir, self.position });
        var sf: f32 = if (self.velocity < 0) -1.0 else 1.0;
        switch (self.phys_state) {
            .accel => {
                if (self.velocity == 0) {
                    const d = self.last_direction;
                    sf = if (d == .up) -1.0 else 1.0;
                }
                const absv = @fabs(self.velocity);
                if (absv > self.speed_limit) unreachable;
                const max_dt = (self.speed_limit - absv) / accel;

                if (max_dt <= dt) {
                    self.position += sf * 0.5 * accel * pow(max_dt, 2) + (self.velocity * max_dt);
                    self.velocity = self.speed_limit * sf;

                    self.phys_state = .constant;
                    self.update_phys(dt - max_dt);
                    return;
                } else {
                    self.position += (sf * 0.5 * accel * (dt * dt)) + (self.velocity * dt);
                    self.velocity += sf * dt * accel;
                }
            },
            .brake => {
                const absv = @fabs(self.velocity);
                if (absv == 0) unreachable;
                if (absv > self.speed_limit) unreachable;
                const max_dt = (absv) / brake;
                //self.last_direction = if (self.velocity > 0) .down else .up;

                if (max_dt <= dt) {
                    self.position += sf * 0.5 * -brake * pow(max_dt, 2) + (self.velocity * max_dt);
                    self.velocity = 0;
                    self.phys_state = .stopped;
                    self.update_phys(dt - max_dt);
                } else {
                    self.position += sf * 0.5 * -brake * pow(dt, 2) + (self.velocity * dt);
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
                    self.update_phys(dt);
                    return;
                }

                //will we overshoot the brake distance in this dt, yes then apply our vel and
                {
                    const t = (self.speed_limit / brake);
                    const brake_dist = (0.5 * (-brake) * (t * t)) + (self.speed_limit * t);
                    if (self.target_floor != null) { //Are there any floors we could stop at before our current_target
                        switch (self.last_direction) {
                            .up => {
                                if (self.getNextCallInDirection(self.position - brake_dist, self.last_direction)) |call| {
                                    if (call > self.target_floor.?) {
                                        self.target_floor = call;
                                        self.update_phys(dt);
                                        return;
                                    }
                                }
                            },
                            .down => {
                                if (self.getNextCallInDirection(self.position + brake_dist, self.last_direction)) |call| {
                                    if (call < self.target_floor.?) {
                                        self.target_floor = call;
                                        self.update_phys(dt);
                                        return;
                                    }
                                }
                            },
                        }
                    }

                    const dxt = @fabs(target_pos - self.position);
                    const dist_before_brake = dxt - brake_dist;

                    const maxdt = (dist_before_brake / self.speed_limit);
                    if (dt > maxdt) {
                        self.position += self.velocity * maxdt;
                        self.phys_state = .brake;
                        self.update_phys(dt - maxdt);
                        return;
                    }
                }

                self.position += self.velocity * dt;
            },
            .stopped => {
                {
                    //Determine what floor we are parked at;
                    const nearest_i = getNearestFloor(self.floor_map.items, self.position);
                    const abs_p = getAbsoluteFloorPosition(self.floor_map.items, nearest_i);
                    self.parked_at = if (@fabs(self.position - abs_p) < leveling_error) nearest_i else null;
                }

                if (self.velocity != 0) unreachable;
                const rdt = blk: {
                    if (self.stop_timer + dt > stop_time) {
                        defer self.stop_timer = 0;
                        break :blk stop_time - self.stop_timer;
                    } else {
                        self.stop_timer += dt;
                        return;
                    }
                };

                if (self.target_floor) |target_floor| {
                    const absfloor = getAbsoluteFloorPosition(self.floor_map.items, target_floor);
                    if (@fabs(absfloor - self.position) < leveling_error) { //TODO is this needed?
                        self.hall_calls_up.unset(self.target_floor.?);
                        self.hall_calls_down.unset(self.target_floor.?);
                        self.cab_calls.unset(self.target_floor.?);
                        self.target_floor = null;
                        self.update_phys(rdt);
                        return;
                    }

                    const floor_pos = getAbsoluteFloorPosition(self.floor_map.items, target_floor);
                    self.speed_limit = calculate_speed_limit(@fabs(self.position - floor_pos));

                    self.phys_state = .accel;
                    self.last_direction = if (absfloor > self.position) .down else .up;
                    self.update_phys(rdt);
                } else {
                    //Determine what velocity curve to use and set max_v accordingly
                    if (self.getNextCallInDirection(self.position, self.last_direction)) |next_call| {
                        self.target_floor = next_call;

                        self.update_phys(rdt);
                        return;
                    } else if (self.getNextCallInDirection(self.position, self.last_direction.invert())) |other_dir_call| {
                        self.target_floor = other_dir_call;
                        self.last_direction = self.last_direction.invert();

                        self.update_phys(rdt);
                        return;
                    }
                }
            },
        }
    }
};

test "Elevator common" {
    const floors = [_]Elevator.FloorMapItem{ .{}, .{ .rel_y = 0.5 }, .{ .rel_y = 2 }, .{}, .{} };
    var p: f32 = 0;
    for (floors, 0..) |floor, i| {
        p += floor.rel_y;
        std.debug.print("index: {d}, abs_pos: {d}\n", .{ i, p });
    }
    const index = Elevator.getNearestFloorIndexDir(&floors, 3.5, .down);
    std.debug.print("Index {any}\n", .{index});

    const di: f32 = 4;
    const po = Elevator.getNearestFloor(&floors, di);
    std.debug.print("{d}, {d}\n", .{ po, di });
}

pub fn draw_hall_call_indicator(draw: *Dctx, pos: Vec2f, up: bool, down: bool) void {
    draw.triangle(pos, .{ .x = pos.x - 10, .y = pos.y - 20 }, .{ .x = pos.x - 20, .y = pos.y }, if (up) Colors.Orange else Colors.White);
    draw.triangle(.{ .x = pos.x, .y = pos.y + 5 }, .{ .x = pos.x - 20, .y = pos.y + 5 }, .{ .x = pos.x - 10, .y = pos.y + 5 + 20 }, if (down) Colors.Orange else Colors.White);
}

pub fn draw_cab_call_indicator(win: *const graph.SDL.Window, draw: *Dctx, pos: Vec2f, cab_calls: *const BitSetT, elevator: *Elevator) void {
    const w = 5;

    const wt = 20;
    const ht = 20;
    const pad = 2;

    var i: usize = 0;
    while (i < cab_calls.unmanaged.bit_length) : (i += 1) {
        const rect: Rect = .{
            .x = pos.x + @as(f32, @floatFromInt(i % w)) * @as(f32, @floatFromInt(pad + wt)),
            .y = pos.y + @as(f32, @floatFromInt(i / w)) * @as(f32, @floatFromInt(pad * ht)),
            .w = 20,
            .h = 20,
        };
        if (hasMouseClickedInArea(win, rect)) {
            elevator.cabCall(i);
        }
        draw.rect(rect, if (cab_calls.isSet(i)) Colors.Red else Colors.Black);
    }
}

pub fn hasMouseClickedInArea(win: *const graph.SDL.Window, area: Rect) bool {
    if (win.mouse.left == .rising) {
        return area.containsPoint(win.mouse.pos);
    }
    return false;
}

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    const alloc = gpa.allocator();

    var win = try graph.SDL.Window.createWindow("Elevator", .{});
    defer win.destroyWindow();

    var draw = Dctx.init(alloc, win.getDpi());
    defer draw.deinit();

    var windir = try std.fs.cwd().openDir("ratgraph", .{});
    var font = try graph.Font.init(alloc, windir, "fonts/roboto.ttf", 12, win.getDpi(), .{
        .debug_dir = std.fs.cwd(),
    });
    defer font.deinit();

    var xorsh = std.rand.DefaultPrng.init(0);
    var rand = xorsh.random();

    const smoke_location = .{ .floor = @as(usize, 5), .x = @as(f32, 400) };

    var elevator = try Elevator.init(alloc, &[_]Elevator.FloorMapItem{
        .{},
        .{ .rel_y = 0.5 },
        .{ .rel_y = 2 },
        .{},
        .{},
        .{},
        .{},
        .{},
        .{},
        .{},
        .{},
        .{},
    });
    defer elevator.deinit();

    var agents = Vec(Agent).init(alloc);
    defer agents.deinit();
    try agents.append(.{ .x = 300, .floor_i = 3, .state = .standing, .goal_x = 1000, .goal_floor = 5 });
    const num_agents = 10;
    {
        var i: usize = 0;
        const num_floors = elevator.floor_map.items.len;
        const maxx = 1000;
        while (i < num_agents) : (i += 1) {
            try agents.append(.{
                .x = rand.float(f32) * maxx,
                .floor_i = rand.uintLessThan(usize, num_floors),
                .state = .standing,
                .goal_x = rand.float(f32) * maxx,
                .goal_floor = rand.uintLessThan(usize, num_floors),
            });
        }
    }

    while (!win.should_exit) {
        try draw.begin(Colors.Gray, win.screen_dimensions.toF());
        win.pumpEvents();
        elevator.update_phys(0.16);
        for (agents.items) |*item| {
            item.update(0.16, &elevator);
        }
        {
            if (win.keyPressed(.A)) {
                elevator.phys_state = .accel;
            }
            if (win.keyPressed(.B)) {
                elevator.phys_state = .brake;
            }
        }

        {
            draw_cab_call_indicator(&win, &draw, .{ .x = 400, .y = 70 }, &elevator.cab_calls, &elevator);
            var y: f32 = 0;
            const sfy = 70;
            for (elevator.floor_map.items, 0..) |floor, i| {
                const pos = Vec2f{ .x = 40, .y = floor.rel_y * sfy };
                draw_hall_call_indicator(
                    &draw,
                    .{ .x = pos.x, .y = y + (floor.rel_y / 2) * sfy },
                    elevator.hall_calls_up.isSet(i),
                    elevator.hall_calls_down.isSet(i),
                );
                //if (hasMouseClickedInArea( Rec( pos.x - 20,  y + (floor.rel_y / 2) * sfy - 20,  20,  20, )) {
                //    elevator.hallCall(i, .up);
                //}
                //if (hasMouseClickedInArea(Rec( pos.x - 20, y + (floor.rel_y / 2) * sfy + 5, 20, 20 ,)) {
                //    elevator.hallCall(i, .down);
                //}
                const color: u32 = if (i % 2 == 0) Colors.White else Colors.Black;
                draw.rectV(.{ .x = sfy, .y = y }, pos, color);
                draw.textFmt(
                    Vec2f.new(sfy + pos.x + 10, y + floor.rel_y * sfy / 3),
                    "{d}",
                    .{i},
                    &font,
                    12,
                    Colors.DarkGray,
                );
                y += floor.rel_y * sfy;
            }
            draw.rect(Rec(smoke_location.x, Elevator.getAbsoluteFloorPosition(elevator.floor_map.items, smoke_location.floor) * sfy, 40, 40), Colors.Black);

            draw.rectV(.{ .x = sfy + 8, .y = 0 }, .{ .x = 4, .y = elevator.position * sfy }, Colors.Black);
            draw.rectV(.{ .x = sfy, .y = elevator.position * sfy - (sfy / 2) }, .{ .x = 20, .y = sfy / 2 }, Colors.Blue);
            {
                for (agents.items) |agent| {
                    switch (agent.state) {
                        .elevating => {},
                        else => {
                            draw.rectV(.{ .x = agent.x + 68, .y = Elevator.getAbsoluteFloorPosition(elevator.floor_map.items, agent.floor_i) * sfy }, .{ .x = 10, .y = 10 }, Colors.Purple);
                        },
                    }
                }
            }
        }
        draw.textFmt(
            Vec2f.new(0, 0),
            "{any}, {any}, {any}, {any}",
            .{ elevator.phys_state, elevator.last_direction, elevator.target_floor, elevator.parked_at },
            &font,
            12,
            Colors.Black,
        );

        //draw.rect(Rec(0, 0, 1000, 1000), 0xff00ffff);
        try draw.end();
        win.swap();
    }
}
