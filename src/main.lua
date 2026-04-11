
function dump(t)
    print("table")
    for k,v in pairs(t) do
        print("", k, v)
    end
end


function string.splitToNumbers (inputstr, sep)
    if sep == nil then
        sep = "%s"
    end
    local t={}
    for str in string.gmatch(inputstr, "([^"..sep.."]+)") do
        local v = tonumber(str)
        if v ~= 0 then
            table.insert(t, v)
        end
    end
    return t
end


AddressDict={}
function AddressDict:new(o)
    setmetatable(o, AddressDict)
    return o
end

function AddressDict:__tostring()
    local result = "{"
    local first = true
    for k,v in pairs(self) do
        if first then
            first = false
        else
            result = result .. ","
        end
        result = result .. string.format("\"%s\":%s", k, v)
    end
    result = result .. "}"
    return result
end


-- MODBUS

ModbusGW = { available = false }
function ModbusGW:new(address)
    local relays = { }
    for i = 0,31 do
        relays[i] = false
    end
    local o = { modbus_address = address, relays=relays, _available=false }

    setmetatable(o, self)
    return o
end


-- Called by external agents to update the state.
function ModbusGW:updateAttr(key, value)
    if key == "available" then
        if value ~= self._available then
            self._available = value
            print(string.format("modbus[%d]=%s", self.modbus_address, tostring(self)))
        end
    elseif type(key) == "number" then
        if value ~= self.relays[key] then
            self.relays[key] = value
            print(string.format("modbus[%d][%d]=%s", self.modbus_address, key, value))
        end
    end
end

function ModbusGW:__newindex(key, value)
    if type(key) == "number" then
        local success, res = pcall(function()
            modbus_set_coil(self.modbus_address, key, value)
            self:update()
        end)
        if not success then
            print("Error setting relay", key, res)
        end
    else
        error(string.format("Unknown key %s", key))
    end
end

function ModbusGW:__index(key)
    if key == "available" then
        return self._available
    elseif type(key) == "number" then
        return self.relays[key]
    end
    return getmetatable(self)[key]
end

function ModbusGW:__tostring ()
    if not self._available then
        return "null"
    else
        local ret = "["
        for addr = 0,#self.relays do
            if addr ~= 0 then
                ret = ret .. ","
            end
            ret = ret .. tostring(self.relays[addr])
        end
        return ret .. "]"
    end
end

function ModbusGW:update ()
    local success, res = pcall(modbus_get_coils, self.modbus_address, 0, 32)
    if success then
        self:updateAttr("available", true)
        for i,v in ipairs(res) do
            self:updateAttr(i-1, v)
        end
    else
        print(string.format("Could not update MODBUS@%d: %s", self.modbus_address, res))
        self:updateAttr("available", false)
    end
end



local daliGetter = {
    brightness = 0x01a0,
    status     = 0x0190,
    min        = 0x01a2,
    max        = 0x01a1,
    faderate   = 0x01a5,
    fadetime   = 0x01a8,
    powerfail  = 0x01a4,
    poweron    = 0x01a3,
    groups    = 0x01c0,
}


local daliSetter = {
    brightness = 0x0000,
    min        = 0x012b,
    max        = 0x012a,
    faderate   = 0x012f,
    fadetime   = 0x012e,
    poweron    = 0x012d,
    powerfail  = 0x012c
}



-- DALI
DaliLight={}
function DaliLight:new (gw, addr)
    local o = {
        gw=gw,
        address=addr,
        _status = 0,
        _available = false,
        _brightness = 0,
        _min = 0,
        _max = 0,
        _poweron = 0,
        _powerfail = 0,
        _faderate = 0,
        _fadetime = 0,
        _groups = 0,
    }
    setmetatable(o, self)
    return o
end

function DaliLight:__index(key)
    if key == "available" or daliGetter[key] ~= nil then
        return self["_" .. key]
    end
    return getmetatable(self)[key]
end

function DaliLight:__newindex (key, val)
    if key == "available" then
        self._available = val
        return
    end
    if key == "brightness" then
        self.gw:raw(val | (self.address << 9))
    elseif key == "groups" then
        -- Groups is managed by a set of add/removes one for each bit.
        -- First make sure we have the up to date values.
        self:update("groups")

        local changed = self._groups ~ val
        print(string.format("New groups are 0x%04x, changed 0x%04x", val, changed))
        local shiftReg = val
        for bit=0,15 do
            if (changed & 0x01) ~= 0 then
                local cmd
                if (shiftReg & 0x01) ~= 0 then
                    print("Adding light to group", bit)
                    cmd = (0x0160 + bit) | (self.address << 9)
                else
                    print("Removing light from group", bit)
                    cmd = (0x0170 + bit) | (self.address << 9)
                end
                self.gw:raw(cmd, true)
            end
            changed = changed >> 1
            shiftReg = shiftReg >> 1
        end
    else
        local code = daliSetter[key]
        if code == nil then
            error("Illegal attribute " .. key)
        end
        self.gw:raw(daliSetter[key] | (self.address << 9))
    end
    self:update(key, true)
end

function DaliLight:on()
    -- Recall last level
    local res = self.gw:raw((self.address << 9) | 0x10a)
    self:update("brightness")

end

function DaliLight:off()
    -- Off
    local res = self.gw:raw((self.address << 9) | 0x100)
    self:update("brightness")
end


function DaliLight:toggle()
    if self.brightness == 0 then
        self:on()
    else
        self:off()
    end
end


function DaliLight:fade(direction)
    -- Fade is done via a raw DALI command
    local cmd = self.address << 9
    if direction < 0 then
        cmd = cmd | 0x107
    else
        cmd = cmd | 0x108
    end
    local success, res = pcall(function()
        self.gw:raw(cmd)
        self:update("brightness")
    end)
    if not success then
        print("Could not execute fade", res)
    end
end

function DaliLight:update(attr, forcePrint)
    if attr == nil then
        self:update("brightness")
        self:update("status")
        self:update("min")
        self:update("max")
        self:update("poweron")
        self:update("powerfail")
        self:update("faderate")
        self:update("fadetime")
        self:update("groups")
        return;
    end

    local cmd = daliGetter[attr]
    if cmd == nil then
        error(string.format("Invalid attribute %s", attr))
    end
    local fullCmd = cmd | (self.address << 9)
    -- print(string.format("sending command 0x%04x", fullCmd))
    local val = self.gw:raw(fullCmd)
    -- print("Result is ", val)
    if val == nil then
        self.available = false
    else
        self.available = true

        if attr == "groups" then
            -- We also need groups 8-15
            fullCmd = fullCmd + 1
            local highByte = self.gw:raw(fullCmd)
            if highByte ~= nil then
                self:updateAttr(attr, val | (highByte << 8), forcePrint)
            end
        else
            self:updateAttr(attr, val, forcePrint)
        end
    end
end



function DaliLight:updateAttr(key, val, forcePrint)
    local shouldPrint = forcePrint or false
    local realattr = "_" .. key

    if self[realattr] ~= nil then
        if self[realattr] ~= val then
            self[realattr] = val
            shouldPrint = true
        end
    end

    if shouldPrint then
        if key == "available" then
            print(string.format("dali[%s][%d]=%s", self.gw.modbus_address, self.address, self))
        else
            print(string.format("dali[%s][%d].%s=%s", self.gw.modbus_address, self.address, key, val))
        end
    end
end

function DaliLight:__tostring ()
    if not self._available then
        return "null"
    end
    return string.format("{\"brightness\": %d, \"status\": %d, \"min\": %d, \"max\": %d, \"poweron\": %d, \"powerfail\": %d, \"faderate\":%d, \"fadetime\":%d, \"groups\": %d }",
        self._brightness or 0,
        self._status or 0,
        self._min or 0,
        self._max or 0,
        self._poweron or 0,
        self._powerfail or 0,
        self._faderate or 0,
        self._fadetime or 0,
        self._groups or 0
    )
end





DaliGW = { }
function DaliGW:new(address)
    local lights = {}
    local o = { modbus_address = address, lights = lights, _available = false }
    for i = 0,63 do
        lights[i] = DaliLight:new(o, i)
    end

    setmetatable(o, self)
    return o
end

function DaliGW:__index(key)
    if key == 'available' then
        return self._available
    end
    if type(key) == 'number' and key >= 0 and key < 64 then
        -- if not self._available then
        --     error("attempt to fetch light from unavailable gateway")
        -- end
        return self.lights[key]
    end
    return getmetatable(self)[key]
end

function DaliGW:__newindex(key, val)
    if key == 'available' then
        if val ~= self._available then
            self._available = val
            print(string.format("dali[%d]=%s", self.modbus_address, tostring(self)))
        end
    else
        error("Can not change any attribtue other than available on a gateway")
    end
end

function DaliGW:raw(cmd, sendtwice)
    return modbus_dali(self.modbus_address, cmd, sendtwice)
end

-- Sends a command and treats _any_ response, including a malformed one, as a Yes.
function DaliGW:yesno(cmd)
    local success, res = pcall(modbus_dali, self.modbus_address, cmd, false)
    if success then
        return res ~= nil
    else
        if res == "Malformed response" or res == "Slave Device Failure" or string.match(res, "Pulse") then
            -- We treat a bus collision as a Yes
            return true
        end
        error(res)
    end
end


function DaliGW:anyDaliDevicesLessThanOrEqualTo(val)
    -- Set Search Address H
    self:raw(0xb100 | ((val >> 16) & 0xff))
    -- Set Search Address M
    self:raw(0xb300 | ((val >> 8) & 0xff))
    -- Set Search Address L
    self:raw(0xb500 | (val & 0xff))
    -- Compare - Will return a value if true, or None/0xFF otherwise.
    return self:yesno(0xa900)
end

function DaliGW:findLowestRandomAddress(low, high)
    -- As a sanity check, make sure there's at least one device in initialisation mode, otherwise we'll be looking for ages.
    local lessThanHigh = self:anyDaliDevicesLessThanOrEqualTo(high)
    if not lessThanHigh then
        return nil
    end

    while low <= high do
        local testPoint = (low + high) // 2
        print(string.format("Search %06x - %06x - %06x", low, testPoint, high))
        if self:anyDaliDevicesLessThanOrEqualTo(testPoint) then
            if low == testPoint then
                return low
            end
            high = testPoint
        else
            -- It must be above the middle value
            low = testPoint + 1
        end
    end
    return nil
end


function DaliGW:clear_all_addresses()
    print("Clearing all existing Short IDs")
    -- Set DTR0 to 255
    self:raw(0xa3ff)
    -- Broadcast clear Short address
    self:raw(0xff80, true)
end

function DaliGW:find_available_addresses()
    avail = {}

    print("checking for available device IDs");
    for addr=0,63 do
        -- Query Control Gear Present for each address.
        local present = self:yesno(0x0191 | (addr << 9))
        if present then
            print(string.format("Address %d is already used", addr))
        else
            table.insert(avail, addr)
            print(string.format("%d addresses available for assignment", #avail))
        end
    end
    return avail
end

function DaliGW:initialise(address)
    local initCmd;
    if type(address) == "number" then
        if address < 0 or address > 63 then
            error("Invalid address")
        end
        print("Targetting address", address)
        -- Target a specific address
        initCmd = 0xa501 | (address << 1)
    elseif address == true then
        print("Targeting all devices (Broadcast)");
        -- Broadcast
        initCmd = 0xa500
    elseif address == false or address == nil then
        print("Targeting unaddressed devices");
        -- Those devices that are not yet addressed
        initCmd = 0xa5ff
    else
        error("Illegal address argument")
    end
    print(string.format("setting devices into initialise mode with command 0x%04x", initCmd))
    self:raw(initCmd, true)
end

function DaliGW:terminate()
    print("Termninating previous provision, if any")
    self:raw(0xa100)
end

function DaliGW:randomise() 
    print("Randomising long addresses")
    self:raw(0xa700, true);
    coroutine.delay(100)
end

function DaliGW:programShortAddress(addr)
    self:raw(0xb701 | (addr << 1), true)
end

function DaliGW:verifyShortAddress(addr)
    return self:yesno(0xb901 | (addr << 1), true)
end

function DaliGW:withdraw()
    -- Withdraw the device we just assigned
    self:raw(0xab00)
end


function DaliGW:provision(address, dryrun)
    if address == nil then
        error("Address parameter (#1) must be boolean or a number between 0 and 63 inclusive")
    end
    if address == true then
        if dryrun then
            print("Not clearing addresses, because of dryrun")
        else
            print("Clearing all previous addresses")
            self:clear_all_addresses()
        end 
    end
    local avail = self:find_available_addresses()
    self:terminate()
    self:initialise(address)
    self:randomise()


    local lowestRandomDeviceIdFound = -1
    local addressIdx = 0
    local devicesAdded = 0
    while lowestRandomDeviceIdFound ~= nil do
        lowestRandomDeviceIdFound = self:findLowestRandomAddress(lowestRandomDeviceIdFound + 1, 0xFFFFFF)

        if lowestRandomDeviceIdFound ~= nil then
            if #avail == 0 then
                error(string.format("Ran out of available addresses after %d items", addressIdx))
            end
            local deviceAddress = table.remove(avail, 1)
            print(string.format("Found device at random address %06x. Assigning address %d", lowestRandomDeviceIdFound, deviceAddress))

            if dryrun then
                print("Not assigning address as we are performing a dry run")
            else
                self:programShortAddress(deviceAddress)
                if not self:verifyShortAddress(deviceAddress) then
                    error(string.format("Device %06x did not verify its address assignment of %d", lowestRandomDeviceIdFound, deviceAddress))
                end
            end
            self:withdraw()
            devicesAdded = devicesAdded + 1
        else
            print("No more devices found")
        end
    end
    print(string.format("Provisioning Complete - found %d devices", devicesAdded))
    coroutine.delay(100)
    -- Do a full scan of all devices again.
    self:update()
end

function DaliGW:update ()
    for address=0,63 do
        local light = self[address]
        success, res = pcall(light.update, light)
        self.available = success

        if not success then
            print("error updating dali device", address, res)
            break
        end
    end
    -- for offset=0,256,64 do
    --     local success, res = pcall(modbus_get_holding_registers, self.modbus_address, offset, 64)

    --     if success then
    --         self.available = true
    --         for addr=0,63 do
    --             self[addr]:updateReg(offset, res[addr+1])
    --         end
    --     else
    --         print(string.format("Could not update DALI@%d: %s", self.modbus_address, res))
    --         self.available = false
    --         break;
    --     end
    -- end
end


function DaliGW:__tostring ()
    if not self._available then
        return "null"
    end
    local res = "["
    for addr=0,63 do
        if addr ~= 0 then
            res = res .. ","
        end
        res = res .. tostring(self[addr])
    end
    return res .."]"
end



function is_on(device, address)
    if device == 1 then
        return modbus[device][address]
    else
        local level = dali[device]:raw(0x01a0 | (address << 9));
        return level ~= nil and level > 0
    end
end

function set_device(device, address, value)
    if device == 1 then
        modbus[device][address] = value
    else
        local l = dali[device][address]
        if value then
            l:on()
        else
            l:off()
        end
    end
end


bindings = {
    _real = {},
    read = function()
        local fp = io.open("/bindings.txt", "r")
        if fp == nil then
            print("No Bindings file... Using defaults")
            for i=0,24*8 do
                bindings._real[i] = {}
            end
            return
        end

        btn = 0
        for line in fp:lines() do
            bindings._real[btn] = string.splitToNumbers(line, ",")
            btn = btn + 1
        end

        fp:close()
    end,

    write = function()
        local fp = io.open("/bindings.txt", "w")
        for addr=0,#bindings-1 do
            fp:write(table.concat(bindings[addr] or {}, ",") .. "\n")
        end
        fp:close()
    end,

    clear = function()
        for addr=0,#bindings-1 do
            bindings._real[addr] = {}
        end
    end,

    add = function(fixture, button, device, address)
        table.insert(bindings._real[fixture*8+button], device*256+address)
    end
}

setmetatable(bindings, {
    __index = function (self, key)
        if type(key) == 'number' and key < (24*8) then
            return bindings._real[key] or {}
        end
        return nil
    end,

    __newindex = function (self, key, val)
        if val == nil then
            val = 0
        end
        if val ~= bindings._real[key] then
            bindings._real[key] = val
            print(string.format("bindings[%d]=[%s]", key, table.concat(val, ",") or "[]"))
            bindings.write()
        end
    end,

    __len = function(self)
        return 24*8
    end,

    __tostring = function(self)
        ret = "["
        for i=0,#self-1 do
            if i ~= 0 then
                ret = ret .. ","
            end
            ret = string.format(ret .. "[" .. table.concat(self[i], ",") .. "]")
        end
        return ret .. "]"
    end,
})
bindings.read()





buttons = {
    pressed = {},
    tasks = {},
    velocity = {},

    press_task = function (btnidx)
        local btnbindings = bindings[btnidx]

        if btnbindings and #btnbindings > 0 then
            local first_binding = btnbindings[1]
            local first_device_id = first_binding >> 8
            local first_address = first_binding & 0xFF

            local is_on = is_on(first_device_id, first_address)
            print("Device ", first_device_id, first_address, "is", is_on)

            for idx, binding in ipairs(bindings[btnidx]) do
                local device_id = binding >> 8
                local address = binding & 0xFF

                print("Setting", device_id, address, "to", not is_on)

                set_device(device_id, address, not is_on)
                -- local gwdev = dali[device_id]
                -- -- Only Dali devices can be faded
                -- if gwdev then
                --     coroutine.delay(800)
                --     local vel = buttons.velocity[btnidx]
                --     if vel then
                --         vel = -vel
                --     else
                --         vel = -1
                --     end
                --     buttons.velocity[btnidx] = vel

                --     local light = gwdev[address]
                --     light:fade(vel)
                --     while true do
                --         coroutine.delay(300)
                --         light:fade(vel)
                --     end
                -- else
                --     gwdev = modbus[device_id]
                --     if gwdev then
                --         -- Toggle the relay
                --         coroutine.schedule(coroutine.create(function() gwdev[address] = not gwdev[address] end))
                --     end
                -- end
            end
        end
        -- We're done, so remove ourselves from the task table for cleanliness
        buttons.tasks[btnidx] = nil
    end,

    release_task = function (btnidx)
        -- local buttonBindings = bindings[btnidx]

        -- if buttons.velocity[btnidx] then
        --     -- We were fading, so reset the velocity if they don't re-press for half a second
        --     coroutine.delay(333)
        --     buttons.velocity[btnidx] = nil
        -- else
        --     for idx, binding in ipairs(buttonBindings) do
        --         local device = binding >> 8
        --         local daligw = dali[device]
        --         -- We only toggle dali devices on release.  Modbus are toggled on press.
        --         if daligw then
        --             -- Run the command in a separate corotine so it can't be interrupted, and it uses up to date values
        --             coroutine.schedule(coroutine.create(function() daligw[binding & 0xFF]:toggle() end))
        --         end
        --     end
        -- end
        -- We're done, so remove ourselves from the task table for cleanliness
        buttons.tasks[btnidx] = nil
    end
}

setmetatable(buttons, {
    __newindex = function(self, key, val)
        if type(key) == 'number' and key >=0 and key < 24*8 and buttons[key] ~= val then
            buttons.pressed[key] = val
            print(string.format("buttons[%d]=%s", key, val))
            print(string.format("Fixture %d, button %d %s", key//8, key%8, val))

            local me, ismain = coroutine.running()
            if ismain then
                print("Button being changed from main, which should never happen")
            else
                -- cancel any existing task that is happening for our button
                local t = buttons.tasks[key]
                -- if t and coroutine.status(t) ~= "dead" then
                --     coroutine.close(t)
                -- end
                buttons.tasks[key] = me
                -- We are already running in a coroutine. process the potentially long running side effect of pressed/released,
                -- with the understanding that we may be cancelled by a subsequent press/release
                if val then
                    buttons.press_task(key)
                else
                    buttons.release_task(key)
                end
            end
        end
    end,

    __index = function (self, key)
        if type(key) == 'number' then
            return buttons.pressed[key] or false
        end
        return nil
    end,

    __len = function(self)
        return 24*8
    end,

    __tostring = function(self)
        ret = "["
        for i=0,#buttons-1 do
            if i ~= 0 then
                ret = ret .. ","
            end
            ret = ret .. tostring(buttons[i])
        end
        return ret .. "]"
    end
})



dali = AddressDict:new{}
dali[2] = DaliGW:new(2)
dali[3] = DaliGW:new(3)

modbus = AddressDict:new{}
modbus[1] = ModbusGW:new(1)

function update()
    for addr,modbusgw in pairs(devices.modbus) do
        print("Scanning modbus", addr)
        modbusgw:update()
    end
    for addr,gwd in pairs(devices.dali) do
        print("Scanning Dali", addr)
        gwd:update()
    end
    return devices
end


devices = AddressDict:new{
    buttons = buttons,
    bindings = bindings,
    dali = dali,
    modbus = modbus,
}

-- Perform an initial scan of the devices, but do it in a co-routine so that this file can be directly run.
coroutine.schedule(coroutine.create(function() return update() end ))
