# encoding: utf-8
import('../type/logger')

define_type :csv do
  extends(:logger)
  composer { s(:logger, :csv) }
end

define_specification :csv do
  extends(:logger)

  # An (ordered) array of the headers for this log file.
  attribute :headers,     list(ruby(:string))

  # The handle for the output file.
  attribute :file_handle, ruby(:*)

  # A flag indicating whether a cell value has been added to the
  # current row.
  attribute :row_started, ruby(:boolean)

  # Constructs a new logger.
  constructor [
    parameter(:output_directory, ruby(:string)),
    parameter(:headers,     list(ruby(:string)), []),
  ] do
    super(output_directory)
    self.headers = headers
    self.file_handle = nil
    self.row_started = false
  end

  # Opens the log file, clearing any previous contents, preparing
  # it for writing.
  method :prepare do
    self.file_handle = File.open(output_directory, "w")
    self.write_header()
    self.row_started = false
  end

  # Writes the header to the log file.
  method :write_header do
    self.file_handle.write(self.headers.join(','))
  end

  # Creates a new row in the log file.
  method :write_row do
    self.file_handle.write("\n")
    self.row_started = false
  end

  # Writes a value to the next cell in the log file.
  method :write_cell, accepts: [
    parameter(:value, ruby(:string))
  ] do
    if self.row_started
      self.file_handle.write(",#{value}")
    else 
      self.file_handle.write("#{value}")
      self.row_started = true
    end
  end

  # Closes the log file.
  method :close do
    self.file_handle.close()
    self.file_handle = nil
  end

end
