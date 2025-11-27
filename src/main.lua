
function dump(t)
    print("table")
    for k,v in pairs(t) do
        print("", k, v)
    end
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
    for i = 0,32 do
        relays[i] = false
    end
    local o = { modbus_address = address, relays=relays, num_relays=32, _available=false }
    relays._gw = o

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
        success, res = pcall(function()
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
        for addr = 0,self.num_relays do
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



local daliAttrMap = {}
daliAttrMap[0] =   {"brightness", "status"}
daliAttrMap[64] =  {"min", "max"}
daliAttrMap[128] = {"faderate", "fadetime"}
daliAttrMap[192] = {"poweron", "powerfail"}
daliAttrMap[256] = {"groups"}
daliAttrMap["brightness"] = 0
daliAttrMap["status"]     = 0
daliAttrMap["min"]        = 64
daliAttrMap["max"]        = 64
daliAttrMap["faderate"]   = 128
daliAttrMap["fadetime"]   = 128
daliAttrMap["poweron"]    = 192
daliAttrMap["powerfail"]  = 192
daliAttrMap["groups"]     = 256


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
        _groups = 0 
    }
    setmetatable(o, self)
    return o
end

function DaliLight:__index(key) 
    if key == "available" or daliAttrMap[key] ~= nil then
        return self["_" .. key]
    end
    return getmetatable(self)[key]
end

function DaliLight:__newindex (key, val) 
    if key == "available" then
        self._available = val
        return
    end

    -- Attributes are represented in pairs. 
    local offset = daliAttrMap[key]


    if offset == nil then
         error(string.format("Unsupported attribute %s", key))
    end

    if string.match(key, "^_") then
        error(string.format("Can not update private attribute %s", key))
    end

    local order = daliAttrMap[offset]
    -- we use combined 16 bit values.  Attributes toggle between the high and low bits.
    local regVal

    if offset == 0 then
        -- Special case for brightness, as the high order byte must be zero when setting
        regVal = val
    elseif #order == 1 then
        regVal = val
    elseif order[1] == key then
        -- value is the low byte
        regVal = val | (self["_"..order[2]] << 8)
    else
        -- value is the high byte
        regVal = (val << 8) | self["_"..order[1]]
    end

    if not modbus_set_holding_register(self.gw.modbus_address, offset + self.address, regVal) then
        error("Setting modbus register failed")
    end

    self:update(offset, true)
end

function DaliLight:toggle()
    modbus_set_coil(self.gw.modbus_address, self.address, self.brightness == 0)
    self:update(0)
end


function DaliLight:fade(direction)
    -- Fade is done via a raw DALI command
    local cmd = self.address << 9
    if direction < 0 then
        cmd = cmd | 0x107
    else
        cmd = cmd | 0x108
    end
    success, res = pcall(function()
        self.gw:raw(cmd)
        self:update(0)
    end)
    if not success then
        print("Could not execute fade", res)
    end
end

function DaliLight:updateReg(regoffset, val, forcePrint)
    local attrs = daliAttrMap[regoffset]
    if attrs == nil then
        error("Invalid register offset")
    end

    if val ~= nil then
        -- A 255 brightness indicates no device is present at that index.
        if regoffset == 0 then
            local available = (val & 0xFF) ~= 0xFF
            self:updateAttr("available", available)
            if not available then
                val = val & 0xFF00
            end
        end

        if #attrs == 1 then
            self:updateAttr(attrs[1], val, forcePrint)
        else
            self:updateAttr(attrs[1], val & 0xFF, forcePrint)
            self:updateAttr(attrs[2], val >> 8, forcePrint)
        end
    else
        self:updateAttr("available", false)
    end
end

function DaliLight:update(regoffset, forcePrint)
    if regoffset == nil then
        self:update(0)
        -- Don't bother getting the other registers if we failed to find the device
        if self.available then 
            self:update(64)
            self:update(128)
            self:update(192)
            self:update(256)
        end
        return
    end

    local addr = regoffset+self.address
    local success, res = pcall(modbus_get_holding_registers, self.gw.modbus_address, addr, 1)
    if success then 
        self:updateReg(regoffset, res[1], forcePrint)
    else
        error(string.format("Timeout or other error while fetching holding register %d: %s", addr, res))
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
            print(string.format("dali[%s][%d]=%s", self.gw.modbus_address, self.address, val))
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
        self._brightness,
        self._status,
        self._min,
        self._max,
        self._poweron,
        self._powerfail,
        self._faderate,
        self._fadetime,
        self._groups
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

function DaliGW:anyDaliDevicesLessThanOrEqualTo(val)
    -- Set Search Address H
    self:raw(0xb100 | ((val >> 16) & 0xff))
    -- Set Search Address M
    self:raw(0xb300 | ((val >> 8) & 0xff))
    -- Set Search Address L
    self:raw(0xb500 | (val & 0xff))
    -- Compare - Will return a value if true, or None/0xFF otherwise.
    return self:raw(0xa900) == 0xFF
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
        local present = self:raw(0x0191 | (addr << 9))
        if present == nil then
             print("DALI Gateway is not responding.  Assuming it is missing");
             return avail;
        elseif present == -10 then
            table.insert(avail, addr)
            print(string.format("%d addresses available for assignment", #avail))
        else 
            print(string.format("Address %d is already used", addr))
        end
    end
    return avail
end

function DaliGW:provision(full)
    if full == nil then
        full = true
    end
    if full then
        self:clear_all_addresses()
    end
    local avail = self:find_available_addresses()
    -- Terminate any existing provisioning processes.
    print("Termninating previous provision, if any")
    self:raw(0xa100)

    -- Initialise - with either None (0xFF) if incremental, or Broadcast if full
    print("Initialising")
    self:raw(full and 0xa500 or 0xa5ff, true)
    
    print("Randomising long addresses")
    self:raw(0xa700, true);

    -- Give the devices a moment to randomise...
    coroutine.delay(100)

    
    local lowestRandomDeviceIdFound = -1
    local addressIdx = 0
    local devicesAdded = 0
    while lowestRandomDeviceIdFound ~= nil do
        lowestRandomDeviceIdFound = self:findLowestRandomAddress(lowestRandomDeviceIdFound + 1, 0xFFFFFF)

        if lowestRandomDeviceIdFound ~= nil then
            if #avail == 0 then
                print(string.format("Ran out of available addresses after %d items", addressIdx))
                return
            end
            local deviceAddress = table.remove(avail, 1)
            print(string.format("Found device at random address %06x. Assigning address %d", lowestRandomDeviceIdFound, deviceAddress))

            -- Program Short Address
            self:raw(0xb701 | (deviceAddress << 1), true)

            -- Verify Short Address
            if self:raw(0xb901 | (deviceAddress << 1), true) ~= 0xFF then
                print(string.format("Device %06x did not verify its address assignment of %d", lowestRandomDeviceIdFound, deviceAddress))
                return
            end
            -- Withdraw the device we just assigned
            self:raw(0xab00)
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
    for offset=0,256,64 do
        local success, res = pcall(modbus_get_holding_registers, self.modbus_address, offset, 64)

        if success then
            self.available = true
            for addr=0,63 do
                self[addr]:updateReg(offset, res[addr+1])
            end
        else
            print(string.format("Could not update DALI@%d: %s", self.modbus_address, res))
            self.available = false
            break;
        end
    end
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




bindings = { 
    _real = {},
    read = function()
        local fp = io.open("/bindings.txt", "r")
        if fp == nil then
            print("No Bindings file... Using defaults")
            for i=0,24*8 do
                bindings._real[i] = 0
            end
            return
        end

        btn = 0
        for line in fp:lines() do
            bindings._real[btn] = tonumber(line)
            btn = btn + 1
        end

        fp:close()
    end,

    write = function()
        local fp = io.open("/bindings.txt", "w")
        for addr=0,#bindings-1 do
            fp:write(string.format("%d\n", bindings[addr] or 0))
        end
        fp:close()
    end,
}

setmetatable(bindings, { 
    __index = function (self, key)
        if type(key) == 'number' and key < (24*8) then
            return bindings._real[key] or 0
        end
        return nil
    end,

    __newindex = function (self, key, val)
        if val == nil then
            val = 0
        end
        if val ~= bindings._real[key] then
            bindings._real[key] = val
            print(string.format("bindings[%d]=%d", key, val or 0))
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
            binding = self[i]
            ret = ret .. tostring(binding)
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
        local binding = bindings[btnidx]

        -- Only Dali devices can be faded
        if binding then 
            local device_id = binding >> 8
            local address = binding & 0xFF
            local gwdev = dali[device_id]
            if gwdev then 
                coroutine.delay(800)
                local vel = buttons.velocity[btnidx]
                if vel then
                    vel = -vel
                else 
                    vel = -1
                end
                buttons.velocity[btnidx] = vel
                
                local light = gwdev[address]
                light:fade(vel)
                while true do
                    coroutine.delay(300)
                    light:fade(vel)
                end
            else
                gwdev = modbus[device_id]
                if gwdev then
                    coroutine.schedule(coroutine.create(function() gwdev[address]:toggle() end))
                end
            end
        end
        -- We're done, so remove ourselves from the task table for cleanliness
        buttons.tasks[btnidx] = nil
    end,

    release_task = function (btnidx)
        local binding = bindings[btnidx]

        if buttons.velocity[btnidx] then
            -- We were fading, so reset the velocity if they don't re-press for half a second
            coroutine.delay(333)
            buttons.velocity[btnidx] = nil
        elseif binding ~= 0 then
            local device = binding >> 8
            local daligw = dali[device]
            -- We only toggle dali devices on release.  Modbus are toggled on press.
            if daligw then
                -- Run the command in a separate corotine so it can't be interrupted, and it uses up to date values
                coroutine.schedule(coroutine.create(function() daligw[binding & 0xFF]:toggle() end))
            end
        end
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
                if t and coroutine.status(t) ~= "dead" then
                    coroutine.close(t)
                end
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