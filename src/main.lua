
function dump(t)
    print("table")
    for k,v in pairs(t) do
        print("", k, v)
    end
end

ObjectList={}
function ObjectList:new(sz)
    local o = { n = sz }
    setmetatable(o, ObjectList)
    return o
end


function ObjectList:__tostring()
    local result = "["
    for idx=0,self.n do
        if idx ~= 0 then
            result = result .. ","
        end
        result = result .. tostring(self[idx])
    end
    result = result .. "]"
    return result
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
        modbus_set_coil(self.modbus_address, key, value)
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
            if ret ~= 0 then
                ret = ret .. ","
            end
            ret = ret .. self.relays[addr]
        end
        return ret .. "]"
    end
end

function ModbusGW:update ()
    res = modbus_get_coils(self.modbus_address, 0, 32)
    if res ~= nil then
        self:updateAttr("available", true)
        for i,v in ipairs(res) do
            self:updateAttr(i-1, v)
        end
    else
        self:updateAttr("available", false)
    end
end



local daliAttrMap = {}
daliAttrMap[64] = {"min", "max"}
daliAttrMap[128] = {"faderate", "fadetime"}
daliAttrMap[192] = {"poweron", "powerfail"}
daliAttrMap[256] = {"groups"}


-- DALI
DaliLight={}
function DaliLight:new (gw, addr)
    local o = { 
        gw=gw, 
        address=addr, 
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
    if key == 'brightness' then
        return self._brightness
    end
    if key == 'available' then
        return self._available
    end
    return getmetatable(self)[key]
end

function DaliLight:__newindex (key, val) 
    if key == 'brightness' then
        modbus_set_holding_register(self.gw.modbus_address, self.address, val)
        self:update()
    else
        error(string.format("unknown attribute", key))
    end
end

function DaliLight:toggle()
    modbus_set_coil(self.gw.modbus_address, self.address, self.brightness == 0)
    self:update()
end


function DaliLight:fade(direction)
    -- Fade is done via a raw DALI command
    local cmd = self.address << 9
    if direction < 0 then
        cmd = cmd | 0x107
    else
        cmd = cmd | 0x108
    end
    self.gw:raw(cmd)
    self:update(true)
end

function DaliLight:update(onlybrightness)
    local status
    repeat
        local res = modbus_get_holding_registers(self.gw.modbus_address, self.address, 1)
        if res ~= nil and res[1] ~= 0xFF then
            self:updateAttr("available", true)
            self:updateAttr("brightness", res[1] & 0xFF)
            status = res[1] >> 8
        else
            self:updateAttr("available", false)
        end
    until res == nil or (status & 0x10) == 0
end



function DaliLight:updateAttr(key,val)
    local changed = false
    local realattr = "_" .. key

    if self[realattr] ~= nil then
        if self[realattr] ~= val then
            self[realattr] = val
            changed = true
        end
    end

    if changed then
        print(string.format("dali[%s].lights[%d].%s=%s", self.gw.modbus_address, self.address, key, val))
    end
end


function DaliLight:__tostring ()
    if not self._available then
        return "null"
    end
    return string.format("{\"brightness\": %d, \"min\": %d, \"max\": %d, \"poweron\": %d, \"powerfail\": %d, \"faderate\":%d, \"fadetime\":%d, \"groups\": %d }", 
        self._brightness,
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
    local lights = ObjectList:new(64)
    local o = { modbus_address = address, lights = lights, _available = false }
    for i = 0,lights.n-1 do
        lights[i] = DaliLight:new(o, i)
    end

    setmetatable(o, self)
    return o
end

function DaliGW:__index(key)
    if key == 'available' then
        return self._available
    end
    return getmetatable(self)[key]
end

function DaliGW:__newindex(key, val) 
    if key == 'available' then
        if val ~= self._available then
            self._available = val
            print(string.format("dali[%d].available=%s", self.modbus_address, self._available))
        end
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
        local testPoint = math.floor((low + high) / 2)
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
    local res = modbus_get_holding_registers(self.modbus_address, 0, 64)

    if res ~= nil then 
        self.available = true
        for i,regval in ipairs(res) do
            local brightness = regval & 0xFF
            local status = regval >> 8

            local light = self.lights[i-1]
            if brightness == 0xFF then
                light:updateAttr("available",  false)
            else
                light:updateAttr("available", true)
                light:updateAttr("brightness", brightness)
            end
        end

        for baseaddr, attrnames in pairs(daliAttrMap) do
            local res = modbus_get_holding_registers(self.modbus_address, baseaddr, 64)
            if res ~= nil then
                for i,regval in ipairs(res) do
                    local light = self.lights[i-1]
                    if #attrnames == 1 then
                        light:updateAttr(attrnames[1], regval)
                    else
                        local low = regval & 0xFF
                        local high = regval >> 8
                        light:updateAttr(attrnames[1], low)
                        light:updateAttr(attrnames[2], high)
                    end
                end
            end
        end
    else
        self.available = false
    end
end


function DaliGW:__tostring ()
    if not self._available then
        return "null"
    end
    return "{\"lights\":" ..  tostring(self.lights) .. "}"
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
            fp:write(string.format("%d\n", bindings[addr]))
        end
        fp:close()
    end,
}

setmetatable(bindings, { 
    __index = function (self, key)
        return bindings._real[key]
    end,

    __newindex = function (self, key, val)
        if val == nil then
            val = 0
        end
        if val ~= bindings._real[key] then
            bindings._real[key] = val
            if val == 0 then
                print(string.format("bindings[%d]=null", key))
            else
                print(string.format("bindings[%d]=%d", key, val))
            end
            bindings.write()
        end
    end,

    __len = function(self)
        return #bindings._real
    end,

    __tostring = function(self)
        ret = "["
        for i=0,#self-1 do
            if i ~= 0 then
                ret = ret .. ","
            end
            binding = self[i]
            if binding == nil then
                ret = ret .. "null"
            else
                ret = ret .. tostring(binding)
            end
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
                
                local light =gwdev.lights[address]
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
                coroutine.schedule(coroutine.create(function() daligw.lights[binding & 0xFF]:toggle() end))
            end
        end
        -- We're done, so remove ourselves from the task table for cleanliness
        buttons.tasks[btnidx] = nil
    end,
}
setmetatable(buttons, {
    __newindex = function(self, key, val)
        if buttons[key] ~= val then
            buttons.pressed[key] = val
            print(string.format("buttons[%d]=%s", key, val))

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
    end,

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


coroutine.schedule(coroutine.create(function() update() end ))
