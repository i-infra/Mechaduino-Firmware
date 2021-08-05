import asyncio
import re
import sys
import time

import numpy as np
import pandas as pd

import serial
import serial.tools.list_ports

import bokeh.plotting
import bokeh.io
import bokeh.layouts
import bokeh.driving

import scipy.signal

def find_arduino(port=None):
    """Get the name of the port that is connected to Arduino."""
    if port is None:
        ports = serial.tools.list_ports.comports()
        for p in ports:
            if p.manufacturer is not None and "Arduino" in p.manufacturer:
                port = p.device
    return port

def handshake_arduino(
    arduino, sleep_time=1, print_handshake_message=False, handshake_code=0
):
    """Make sure connection is established by sending
    and receiving bytes."""
    # Close and reopen
    arduino.close()
    arduino.open()
    
    # Chill out while everything gets set
    time.sleep(sleep_time)

    # Set a long timeout to complete handshake
    timeout = arduino.timeout
    arduino.timeout = 2

    # Read and discard everything that may be in the input buffer
    _ = arduino.read_all()

    # Read in what Arduino sent
    handshake_message = arduino.read_until()

    # Print the handshake message, if desired
    print("Handshake message: " + handshake_message.decode())

    # Reset the timeout
    arduino.timeout = timeout
    
    return True

def connectSerial():
    connected = False

    while(connected ==  False):
        try:
            port = find_arduino()
            arduino = serial.Serial(port, baudrate=115200)
            connected = handshake_arduino(arduino)
        except:
            pass
    
    return arduino

def read_all_newlines(ser, read_buffer=b"", n_reads=4):
    """Read data in until encountering newlines.

    Parameters
    ----------
    ser : serial.Serial() instance
        The device we are reading from.
    n_reads : int
        The number of reads up to newlines
    read_buffer : bytes, default b''
        Previous read buffer that is appended to.

    Returns
    -------
    output : bytes
        Bytes object that contains read_buffer + read.

    Notes
    -----
    .. This is a drop-in replacement for read_all().
    """
    raw = read_buffer
    for _ in range(n_reads):
        raw += ser.read_until()

    return raw

def parse_read(read, ts):
    """Parse a read with time, volage data

    Parameters
    ----------
    read : byte string
        Byte string with comma delimited time/voltage
        measurements.

    Returns
    -------
    ctime : system time
    pos   : list of floats; position in degrees
    eff   : list of floats; effort
    eff_r : list of floats; effort with rolling avg
    setpt : list of floats; the setpoint
    flag  : the control flag register.
    rem   : Remaining, unparsed bytes.
    """
    ctime=[]
    pos=[]
    eff=[]
    eff_r=[]
    setpt=[]
    flag=[]

    # Separate independent measurements
    pattern = re.compile(b"\d+|,")
    raw_list = read.split(b"\n")
    
    for reading in raw_list:
        try:
            reads = reading.split(b',')
            pos.append(float(reads[0]))
            eff.append(float(reads[1]))
            eff_r.append(float(reads[2]))
            setpt.append(float(reads[3]))
            flag.append(reads[4])
            ctime.append(time.time()-ts)
        except:
            pass

    if len(raw_list) == 0:
        return ctime, pos, eff, eff_r, setpt, flag, b""
    else:
        return ctime, pos, eff, eff_r, setpt, flag, raw_list[-1]

async def daq_stream_async(arduino, data, ts, delay=25, n_trash_reads=4, n_reads_per_chunk=2):
    """Obtain streaming data"""

    # Current streaming state
    stream_on = False
    # Receive data
    read_buffer = [b""]
    # Always streaming
    while True:
        if data["mode"] == "stream":
        # Turn on the stream if need be
            if not stream_on:
                sendOut(arduino, "M111 S1")
                # Read and throw out first few reads
                i = 0
                while i < n_trash_reads:
                    _ = arduino.read_until()
                    i += 1

                stream_on = True

            # Read in chunk of data
            raw = read_all_newlines(arduino, read_buffer=read_buffer[0], n_reads=n_reads_per_chunk)

            # Parse it, passing if it is gibberish
            try:
                ctime, pos, eff, eff_r, setpt, flag, read_buffer[0] = parse_read(raw, ts)
                # Update data dictionary
                data["ctime"]      += ctime
                data["position"]    += pos
                data["effort"]   += eff
                data["effort_r"]  += eff_r
                data["setpoint"] += setpt
                data["flags"] += flag
            except:
                pass
        
        else:
            stream_on = False
            
        # Sleep 80% of the time before we need to start reading chunks
        await asyncio.sleep(0.8 * n_reads_per_chunk * delay / 1000)

def sendOut(arduino, command):
    arduino.write(command.encode('utf-8'))

stream_data = dict(prev_array_length=0, ctime=[], position=[], effort=[], effort_r=[], setpoint=[], flags=[], mode="stop")

def controls():
    save_notice = bokeh.models.Div(
        text="<p>No streaming data saved.</p>", width=100
        )
    acquire = bokeh.models.Toggle(label="stream", button_type="success", width=100)
    save = bokeh.models.Button(label="save", button_type="primary", width=100)
    reset = bokeh.models.Button(label="reset", button_type="warning", width=100)
    file_input = bokeh.models.TextInput(
        title="file name", value="streamed.csv", width=165
        )
    gcode = bokeh.models.TextInput(
        title="GCode Command", value="", width=165
        )
    sendg = bokeh.models.Button(label="Send", button_type="primary", width=100)
    return dict(
        acquire=acquire,
        reset=reset,
        save=save,
        file_input=file_input,
        gcode=gcode,
        sendg=sendg,
        save_notice=save_notice,
    )

def layout(p1, p2, p3, ctrls):
    buttons = bokeh.layouts.row(
        bokeh.models.Spacer(width=30),
        ctrls["acquire"],
        bokeh.models.Spacer(width=5),
        ctrls["reset"],
    )
    cmds = bokeh.layouts.row(
        bokeh.models.Spacer(width=30),
        ctrls["gcode"],
        bokeh.models.Spacer(width=5),
        ctrls["sendg"],
    )
    left = bokeh.layouts.column(p1, p2, p3, buttons, cmds, spacing=15)
    right = bokeh.layouts.column(
        bokeh.models.Spacer(height=50),
        ctrls["file_input"],
        ctrls["save"],
        ctrls["save_notice"],
    )
    return bokeh.layouts.row(
        left, right, spacing=10, margin=(10, 10, 10, 10), background="whitesmoke",
    )

def reset_callback(data, pos_source, r_e_source, set_source, pos_source_ph, r_e_source_ph, set_source_ph, the_controls):
    # Black out the data dictionaries
    data["ctime"] = []
    data["position"] = []
    data["effort"] = []
    data["effort_r"] = []
    data["setpoint"] = []
    data["flags"] = []
    data["prev_array_length"] = 0
    
    # Stop acquiring
    the_controls["acquire"].active = False

    # Reset the sources
    pos_source.data = dict(t=[], p=[])
    pos_source_ph.data = dict(t=[0], p=[0])
    set_source.data = dict(t=[], p=[])
    set_source_ph.data = dict(t=[0], p=[0])
    r_e_source.data = dict(t=[], p=[])
    r_e_source_ph.data = dict(t=[0], p=[0])

def save_callback(data, controls):
    # Convert data to data frame and save
    df = pd.DataFrame(data={"time (s)": data["ctime"], "Position (deg)": data["position"], "Effort (raw)": data["effort"],
                            "Effort (filtered)": data["effort_r"], "Setpoint": data["setpoint"], "Flags": data["flags"]})
    df.to_csv(controls["file_input"].value, index=False)

    # Update notice text
    notice_text = "<p>" + ("Streaming")
    notice_text += f" data was last saved to {controls['file_input'].value}.</p>"
    controls["save_notice"].text = notice_text
    
def sendg_callback(arduino, controls):
    toSend = controls['gcode'].value
    sendOut(arduino, toSend)
    
def disable_controls(controls):
    """Disable all controls."""
    for key in controls:
        controls[key].disabled = True

def shutdown_callback(
    arduino, daq_task, stream_data, stream_controls
):
    # Disable controls
    disable_controls(stream_controls)

    # Stop DAQ async task
    daq_task.cancel()

    # Disconnect from Arduino
    arduino.close()

def stream_callback(arduino, stream_data, new):
    if new:
        stream_data["mode"] = "stream"
        sendOut(arduino, "M111 S1")
    else:
        stream_data["mode"] = "stop"
        sendOut(arduino, "M111 S0")

    arduino.reset_input_buffer()

def stream_update(data, pos_source, r_e_source, set_source, pos_source_ph, r_e_source_ph, set_source_ph, rollover):
    # Update plot by streaming in data
    new_pos_data = {
        "t" : np.array(data["ctime"][data["prev_array_length"] :]),
        "p" : np.array(data["position"][data["prev_array_length"] :]),
    }
    new_r_e_data = {
        "t" : np.array(data["ctime"][data["prev_array_length"] :]),
        "p" : np.array(data["effort_r"][data["prev_array_length"] :]),
    }
    new_set_data = {
        "t" : np.array(data["ctime"][data["prev_array_length"] :]),
        "p" : np.array(data["setpoint"][data["prev_array_length"] :]),
    }
    
    pos_source.stream(new_pos_data, rollover)
    r_e_source.stream(new_r_e_data, rollover)
    set_source.stream(new_set_data, rollover)
    

    # Adjust new phantom data point if new data arrived
    if len(new_pos_data["t"] > 0):
        pos_source_ph.data = dict(t=[new_pos_data["t"][-1]], p=[new_pos_data["p"][-1]])
        r_e_source_ph.data = dict(t=[new_r_e_data["t"][-1]], p=[new_r_e_data["p"][-1]])
        set_source_ph.data = dict(t=[new_set_data["t"][-1]], p=[new_set_data["p"][-1]])
    data["prev_array_length"] = len(data["ctime"])
    
def plot(y_ax_lab, x_ax_lab):
    # Set up plot area
    p = bokeh.plotting.figure(
        frame_width=600,
        frame_height=400,
        x_axis_label=x_ax_lab,
        y_axis_label=y_ax_lab,
        title="streaming data",
        toolbar_location="above",
    )

    # No range padding on x: signal spans whole plot
    p.x_range.range_padding = 0

    # We'll sue whitesmoke backgrounds
    p.border_fill_color = "whitesmoke"

    # Defined the data source
    source = bokeh.models.ColumnDataSource(data=dict(t=[], p=[]))

    p.line(source=source, x="t", y="p")

    # Put a phantom circle so axis labels show before data arrive
    phantom_source = bokeh.models.ColumnDataSource(data=dict(t=[0], p=[0]))
    p.circle(source=phantom_source, x="t", y="p", visible=False)

    return p, source, phantom_source

def dataplotter(arduino, data, daq_task, rollover=400, stream_plot_delay=90,):
    def _app(doc):
        #Plots
        plot_pos, pos_source, pos_source_ph = plot("Position (degrees)","Time (s)")
        plot_r_e, r_e_source, r_e_source_ph = plot("Effort, rolling-avg","Time (s)")
        plot_set, set_source, set_source_ph = plot("Setpoint (degrees or RPM)","Time (s)")
        
        stream_controls = controls()
        stream_layout = layout(plot_pos, plot_r_e, plot_set, stream_controls)
        shutdown_button = bokeh.models.Button(
            label="shut down", button_type="danger", width=100
        )
        shutdown_layout = bokeh.layouts.row(
            bokeh.models.Spacer(width=675), shutdown_button
        )
        app_layout = bokeh.layouts.column(
            stream_layout, shutdown_layout
        )
        
        def _stream_reset_callback(event=None):
            reset_callback(data, pos_source, r_e_source, set_source, pos_source_ph, r_e_source_ph, set_source_ph, stream_controls)
        def _stream_save_callback(event=None):
            save_callback(stream_data, stream_controls)
        def _stream_callback(attr, old, new):
            stream_callback(arduino, stream_data, new)
        def _sendg_callback(event=None):
            sendg_callback(arduino, stream_controls)
        def _shutdown_callback(event=None):
            shutdown_callback(arduino, daq_task, stream_data, stream_controls)
        
        @bokeh.driving.linear()
        def _stream_update(step):
            stream_update(data, pos_source, r_e_source, set_source, pos_source_ph, r_e_source_ph, set_source_ph, rollover)
        
         # Link callbacks
        stream_controls["acquire"].on_change("active", _stream_callback)
        stream_controls["reset"].on_click(_stream_reset_callback)
        stream_controls["save"].on_click(_stream_save_callback)
        stream_controls["sendg"].on_click(_sendg_callback)
        shutdown_button.on_click(_shutdown_callback)

        # Add the layout to the app
        doc.add_root(app_layout)

        # Add a periodic callback, monitor changes in stream data
        pc = doc.add_periodic_callback(_stream_update, stream_plot_delay)

    return _app

ard = connectSerial()
time_start = time.time();
daq_task = asyncio.create_task(daq_stream_async(ard, stream_data, time_start))
app = dataplotter(ard, stream_data, daq_task)
app(bokeh.plotting.curdoc())