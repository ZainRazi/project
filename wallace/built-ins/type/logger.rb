# encoding: utf-8
import('rng')
import('statistics')
import('population')

define_type :logger do
  composer { s(:logger) }
end

define_specification :logger do

  ruby_file('fileutils')
  ruby_file('pathname')

  # The number of iterations between logging requests.
  #attribute :interval, ruby(:integer)

  # The path to the output file directory used by all loggers.
  attribute :output_directory,  ruby(:string)

  # Constructs a new logger.
  constructor [
    parameter(:output_directory, ruby(:string))
  ] do
    self.output_directory = Pathname.new(output_directory).cleanpath.to_s
  end

  # Prepares this logger before the start of the algorithm.
  method :prepare do

  end

  # Creates an entry in the log.
  abstract_method :log

  # This is too EA specific at the moment!
  method :call, accepts: [
    parameter(:rng,         s(:rng)),
    parameter(:statistics,  s(:statistics)),
    parameter(:population,  s(:population))
  ] do
    return log(rng, statistics, population)
  end

  # Closes this logger at the end of the algorithm.
  method :close do

  end

end
