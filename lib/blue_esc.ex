defmodule BlueESC do
  @moduledoc """
  Handles the control and monitoring of 1 Blue Robotics ESC controller via I2C.

  The throttle command is a 16-bit signed integer.
  The sign of the value determines the direction of rotation.
  -32767 (max reverse) to 32767 (max forward)
  """
  use GenServer
  require Logger

  def start(devname, address, class, opts \\ []) do
    GenServer.start(__MODULE__, [devname, address, class], opts)
  end

  @doc false
  def start_link(devname, address, class, opts \\ []) do
    GenServer.start_link(__MODULE__, [devname, address, class], opts)
  end

  @doc false
  def init([devname, address, class]) do
    poll_factor = case class do
      100 -> 6 #12 / 2
      200 -> 7 #14 / 2
      _ -> 6
    end

    #REVIEW: Trap Exit of i2c process?
    {:ok, pid} = I2c.start_link(devname, address)

    #Make sure ESC properly attached
    << _ :: 64, alive :: 8-big >> = read_all_registers(%{i2c_pid: pid})
    case alive do
      0xAB ->
        Logger.debug "ESC Found, Initializing..."
        initialize_controller(pid)
        Logger.debug "ESC Initialized"

        # Read the data registers at rate about 4hz
        :timer.send_interval(250, :update)
        {:ok, %{i2c_pid: pid, rpm: 0, update_time: 0, poll_factor: poll_factor, volts: nil, temp: nil, current: nil, throttle: 0}}
      _ ->
        Logger.debug "Failed to communicate with ESC"
        {:stop, :no_esc_found}
    end
  end

  def get_state(pid) do
    GenServer.call pid, :get_state
  end

  def handle_call(:get_state, state) do
    {:reply, state, state}
  end

  @doc "Sets the controller to stop"
  def stop(pid) do
    GenServer.call pid, :stop
  end

  @doc "Returns the last read RPM"
  def rpm(pid) do
    GenServer.call pid, :get_rpm
  end

  @doc """
    Sets the speed of the controller
    Values: -100 (reverse) to 100 (forward) in persentage
    iex > set_speed(pid, 100)
    {:ok, _} #TODO: get response
  """
  def set_speed(pid, speed_percent) do
    case speed_percent do
      0 -> stop(pid)
      sp ->
        speed =
          327.67 * sp
          |> round
        GenServer.call pid, {:set_speed, speed}
    end
  end

  @doc """
  Returns the last read voltage from the last full update
  """
  def voltage(pid), do: GenServer.call(pid, :last_voltage)
  @doc """
  Reads the ECS registers and returns the voltage
  Note: Advised to use voltage/0 unless the delay (~250ms)
  is unacceptable
  """
  def voltage!(pid), do: GenServer.call(pid, :get_voltage)

  @doc """
  Read curernt of ESC
  """
  def current(pid), do: GenServer.call(pid, :last_current)
  @doc """
  Reads the ECS registers and returns the current
  Note: Advised to use current/0 unless the delay (~250ms)
  is unacceptable
  """
  def current!(pid), do: GenServer.call(pid, :get_current)

  @doc """
  Read temperature of controller
  """
  def temperature(pid), do: GenServer.call(pid, :last_temperature)
  @doc """
  Reads the ECS registers and returns the temperature
  Note: Advised to use tempurature/0 unless the delay (~250ms)
  is unacceptable
  """
  def temperature!(pid), do: GenServer.call(pid, :get_temperature)

  def handle_call(:stop, _from, state) do
    case initialize_controller(state.i2c_pid) do
      :ok -> {:reply, :ok, %{state | throttle: 0}}
      _ -> {:reply, :error, state}
   end
  end

  def handle_call(:get_rpm, _from, state), do: {:reply, state.rpm, state}

  def handle_call({:set_speed, speed}, _from, state) do
    case I2c.write(state.i2c_pid, <<0x0, speed :: 16-little>>) do
      :ok -> {:reply, :ok, %{state | throttle: speed}}
      _ -> {:reply, :error, state}
    end
  end

  def handle_call(:last_voltage, _from, state), do: {:reply, state.volts, state}

  def handle_call(:get_voltage, _from, state) do
    raw_volts = read_two_registers(0x04, state)
    {:reply, compute_voltage(raw_volts), state}
  end

  def handle_call(:last_current, _from, state), do: {:reply, state.current, state}

  def handle_call(:get_current, _from, state) do
    raw_current = read_two_registers(0x08, state)
    {:reply, compute_current(raw_current), state}
  end

  def handle_call(:last_temperature, _from, state), do: {:reply, state.temp, state}

  def handle_call(:get_temperature, _from, state) do
    raw_temp = read_two_registers(0x06, state)
    {:reply, compute_temp(raw_temp), state}
  end

  @doc false
  # The update function reads new data from the ESC. If used, this function
  # must be called at least every 65 seconds to prevent 16-bit overflow of
  # the timer keeping track of RPM. Recommended to call this function at 4-10 Hz
  def handle_info(:update, state) do
    #REVIEW: Hack to keep controller alive
    I2c.write state.i2c_pid, << 0x0, state.throttle :: 16-big >>
    <<
      pulse_count :: 16-big,
      raw_v :: 16-big,
      raw_temp :: 16-big,
      raw_cur :: 16-big,
      _alive :: 8-big
    >> = read_all_registers(state)

    [rpm, time] = compute_rpm pulse_count, state
    volts = compute_voltage raw_v
    temp = compute_temp raw_temp
    cur = compute_current raw_cur
    {:noreply, %{state | rpm: rpm, update_time: time, volts: volts, temp: temp, current: cur}}
  end

  # ECS throttle registers (0x00-0x01) must be initialized with 0
  defp initialize_controller(pid), do: I2c.write(pid, <<0x0, 0 :: 16-big>>)

  defp read_all_registers(state) do
    pid = state.i2c_pid
    I2c.write_read(pid, << 0x02 >>, 9)
  end

  defp read_two_registers(start, state) do
    I2c.write_read(state.i2c_pid, start, 2)
  end

  defp compute_rpm(pulse_count, state) do
    rpm = pulse_count/((:erlang.system_time-state.update_time)/1000.0)*60/state.poll_factor
    [rpm, :erlang.system_time]
  end

  defp compute_voltage(raw_voltage), do: 0.0004921 * raw_voltage

  defp compute_current(raw_current), do: (raw_current - 32767) * 0.001122

  defp compute_temp(raw_temp) do
    resistance = 3300 / (65535 / raw_temp - 1 )

    steinhart = resistance / 10000
    steinhart = :math.log(steinhart)
    steinhart = steinhart / 3900
    steinhart = steinhart + (1.0 / (25 + 273.15))
    steinhart = 1.0 / steinhart
    steinhart - 273.15
  end
end
