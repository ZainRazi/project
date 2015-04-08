# encoding: utf-8
# import('rng')
# import('statistics')
# import('population')

# define_type :logger do
#   composer { s(:logger) }
# end

# define_specification :logger do

#   # The number of generations between logging.
#   attribute :interval,    ruby(:integer)

#   # An (ordered) array of the headers for this log file.
#   attribute :headers,     list(ruby(:string))

#   # The path to the output file.
#   attribute :destination, ruby(:string)

#   # The findle handle for the output file.
#   attribute :file_handle, ruby(:*)

#   # A flag indicating whether a cell value has been added to the
#   # current row.
#   attribute :row_started, ruby(:boolean)

#   # Constructs a new logger.
#   constructor [
#     parameter(:destination, ruby(:string)),
#     parameter(:headers,     list(ruby(:string)), []),
#     parameter(:interval,    ruby(:integer), 1)
#   ] do
#     self.headers = headers
#     self.destination = destination
#     self.interval = interval
#     self.file_handle = nil
#     self.row_started = false
#   end

#   # Opens the log file, clearing any previous contents, preparing
#   # it for writing.
#   method :open do
#     self.file_handle = File.open(destination, "w")
#     self.write_header()
#     self.row_started = false
#   end

#   # Writes the header to the log file.
#   method :write_header do
#     self.file_handle.write(self.headers.join(','))
#   end

#   # Creates a new row in the log file.
#   method :write_row do
#     self.file_handle.write("\n")
#     self.row_started = false
#   end

#   # Writes a value to the next cell in the log file.
#   method :write_cell, accepts: [
#     parameter(:value, ruby(:string))
#   ] do
#     if self.row_started
#       self.file_handle.write(",#{value}")
#     else 
#       self.file_handle.write("#{value}")
#       self.row_started = true
#     end
#   end

#   # Closes the log file.
#   method :close do
#     self.file_handle.close()
#     self.file_handle = nil
#   end

#   # This method is called after each interval and carries out the
#   # process of logging.
#   abstract_method :log, accepts: [
#     parameter(:rng,         s(:rng)),
#     parameter(:statistics,  s(:statistics)),
#     parameter(:population,  s(:population))
#   ]

# end
