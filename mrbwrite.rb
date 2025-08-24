# mrbwrite.rb
# This script is part of the mrbwrite project, which provides a command-line interface
# for interacting with a device over UART. It allows users to send commands to the device,
# such as writing data, executing programs, and resetting the device.

# The script is designed to run indefinitely, processing commands from the user and
# responding with appropriate messages.
# It includes basic error handling and command parsing.
#

# how to convert to bytecode.
#  mrbc --remove-lv -B mrbwrite_bytecode -o mrbwrite_bytecode.c mrbwrite.rb

##
# MrbWrite class
#
# Handles the communication with the device.
# It listens for commands from the user, processes them, and sends responses back.
# The commands include version information, clearing the device, writing data,
# executing programs, resetting the device, and displaying help information.
#
# Usage:
# 1. Initialize the class with a UART device.
# 2. Call the `run` method to start processing commands.
#
class MrbWrite

  VERSION_STRING = "mruby/c v3.3 RITE0300 MRBW1.2"

  ##
  # Initializes the MrbWrite class with a IO device.
  #
  # @param device [UART] The IO device such as UART.
  # @return [void]
  #
  def initialize( device )
    @device = device
  end

  ##
  # Runs the command processing loop.
  # It continuously reads commands from the device, processes them,
  # and sends responses back to the device.
  #
  # @return [void]
  #
  def run()
    ok(VERSION_STRING)

    while true
      begin
        s = @device.gets.chomp
        cmd,param = s.split(" ",2)

        case cmd
        when nil        ; ok("mruby/c")
        when "version"  ; cmd_version()
        when "clear"    ; cmd_clear()
        when "write"    ; cmd_write(param)
        when "reset"    ; cmd_reset()
        when "help"     ; cmd_help()
        when "showprog" ; cmd_showprog()

        when "execute"
          cmd_execute()
          break

        else
          err("Illegal command")
        end
      rescue =>ex
        err(ex.message)
      end
    end
  end

  ##
  # Sends an OK response to the device.
  # Optionally, a message can be included in the response.
  #
  # @param msg [String, nil] Optional message to include in the response.
  # @return [void]
  #
  # Example:
  #   ok("Command executed successfully")
  #
  # This will send a response like "+OK Command executed successfully" to the device.
  #
  # If no message is provided, it will simply send "+OK".
  #
  def ok( msg = nil )
    @device.puts( msg ? "+OK #{msg}\r\n" : "+OK\r\n" )
  end

  ##
  # Sends a DONE response to the device.
  # Optionally, a message can be included in the response.
  # @param msg [String, nil] Optional message to include in the response.
  #
  # @return [void]
  #
  def done( msg = nil )
    @device.puts( msg ? "+DONE #{msg}\r\n" : "+DONE\r\n" )
  end

  ##
  # Sends an error response to the device.
  # Optionally, a message can be included in the response.
  # @param msg [String, nil] Optional message to include in the error response.
  #
  # @return [void]
  #
  def err( msg = nil )
    @device.puts( msg ? "-ERR #{msg}\r\n" : "-ERR\r\n" )
  end

  ##
  # Sends the version string to the device.
  # This method is called when the "version" command is received.
  # It responds with the version information.
  #
  # @return [void]
  #
    def cmd_version()
    ok(VERSION_STRING)
  end

  ##
  # Clears the device's programs.
  #
  # @return [void]
  #
  def cmd_clear()
    if c_clear()
      ok()
    else
      err()
    end
  end

  ##
  # Writes bytecode to the device.
  # It reads the specified number of bytes from the device and writes them to the FLASH ROM.
  #
  # @param param [String] The size of the bytecode to write.
  # @return [void]
  #
  def cmd_write( param )
    size = param.to_i

    # check size
    if size <= 0
      raise "Invalid size"
    end
    ok("Write bytecode.")

    # read data from device
    data = @device.read(size)

    # check 'RITE' magick code.
    if !data.start_with?("RITE")
      raise "No RITE code received."
    end

    # write bytecode to FLASH ROM.
    c_write( data )
    done()

  rescue =>ex
    err(ex.message)
  end

  ##
  # Executes the current program on the device.
  # It sends an OK response to indicate that the command was executed successfully.
  #
  # @return [void]
  #
  def cmd_execute()
    ok()
  end

  ##
  # Resets the device.
  # It sends an OK response and then calls the reset function on the device.
  #
  # @return [void]
  #
  def cmd_reset()
    ok()
    c_reset()
  end

  ##
  # Displays the help message with available commands.
  # It lists all the available commands and their descriptions.
  #
  # @return [void]
  #
  def cmd_help()
    @device.puts( <<EOS )
  version
  clear
  write <byte_size>
  execute
  reset
  help
  showprog
EOS
  end

  ##
  # Displays the current program list on the device.
  # It sends an OK response and then calls the function to show the program.
  #
  # @return [void]
  #
  def cmd_showprog()
    c_showprog()
    done()
  end

end


device = UART.new(1)
device.clear_rx_buffer()
mrbwrite = MrbWrite.new( device )
mrbwrite.run()
