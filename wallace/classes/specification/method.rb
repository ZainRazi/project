# encoding: utf-8
#
# WARNING:
# - For now we only accept method definitions which use "do" procs, and not
#   the curly brace form.
class Wallace::Specification::Method

  # The name of this method.
  attr_reader :name

  # The parameters for this method.
  attr_reader :accepts

  # The return type of this method.
  attr_reader :returns

  # The source code for this method.
  attr_reader :source

  # Constructs a new method.
  #
  # ==== Parameters
  # [+name+]    The name of this method.
  # [+options+] A hash of keyword options for construction.
  # [+&proc+]   An optional block containing the body for this method; must be
  #             provided if the body of the method isn't supplied by the source
  #             keyword option.
  #
  # ==== Options
  # [+returns+] The type that this method returns.
  # [+accepts+] A list of parameters that this method accepts.
  # [+source+]  The source code for the body of this method.
  def initialize(name, options = {}, &proc)
    @name = name
    @returns = options[:returns] || nil
    @proc = proc.nil? ? eval("lambda { #{@source} }") : proc
    @source = proc.nil? ? options[:source] : source_better

    # Check if the list of parameters has been supplied to this method as a
    # parameter set, as is the case when implementing abstract methods.
    if !options[:accepts].nil? && options[:accepts].is_a?(Wallace::Specification::ParameterSet)
      @accepts = options[:accepts]

    # Otherwise construct a parameter set from the list of parameter objects;
    # if no parameters are provided, then construct an empty parameter set.
    else
      @accepts = Wallace::Specification::ParameterSet.new(options[:accepts] || [])
    end

  end

  # Returns the Ruby source code for this method.
  def source_better

    # Find the file and line where the block was declared and extract the
    # contents of the file (from the block definition downwards - don't bother
    # with anything before the definition!).
    file, line_no = @proc.source_location
    src = File.readlines(file)[line_no-1 .. -1].join

    # ----------------------------------------------------------------------
    # Clean up the file to make parsing easier.
    # Based upon code from RubyPlusPlus project.

    # - Remove empty lines.
    src.squeeze("\n")

    # - Remove excess whitespace.
    src.gsub(/\t/, ' ').squeeze(' ')

    # - Remove all comments.
    src.gsub(/#.*/, '')

    # - Remove all leading and trailing whitespace.
    src = src.lines.map { |l| l.strip }.join("\n")

    # - Temporarily extract all strings from the source and store them in a
    #   buffer.
    string_buffer = []
    src.gsub(/\"(\\.|[^\"])*\"/) do |match|
      string_buffer << match
      "$STRING_#{string_buffer.length - 1}$"
    end

    # ----------------------------------------------------------------------

    # Split the source code up into lines to make processing easier.
    src = src.lines.map(&:chomp)

    # Find the line where the block begins and remove all code from that line
    # and above.
    begin
      line = src.shift
    end until line.nil? || line.end_with?('do')

    # Throw an exception if the start of the block cannot be found.
    if src.empty?
      fail("Failed to find valid method definition: must start with a 'do' statement
  without arguments.")
    end

    # Go through each line until the end tag is found.
    end_line = nil
    open_blocks = 1
    src.each_with_index do |line, no|

      # Record the opening of a block statement.
      if  line.match(/^\bif\b/)    || # if statements
          line.match(/\bdo\b/)     || # do statements
          line.match(/^\bdef\b/)   || # def statements
          line.match(/^\bbegin\b/) || # begin statements
          line.match(/^\bfor\b/)   || # for statements
          line.match(/^\buntil\b/)    # until statements
        open_blocks += 1

      # Record the closure of a block statement.
      elsif line.match(/^\bend\b/) # end statements
        open_blocks -= 1
      end

      # Check if the end of the block has been reached.
      if open_blocks == 0
        end_line = no
        break
      end

    end

    # Remove all code from the line where the end tag belongs and below.
    src = src[0 ... end_line].join("\n")

    # Reinsert the removed strings back into the method definition.
    src = src.gsub(/\$STRING_(\d)+\$/) { |n| string_buffer[n[8...-1].to_i] }

    # Return the extracted source code.
    return src

  end

  # Returns a list of the specifications upon which this method depends.
  def dependencies

    # Calculate the dependencies for the return type.
    if returns.nil?
      deps = []
    elsif returns.is_a?(Wallace::Specification)
      deps = [returns]
    elsif [ Wallace::List,
            Wallace::Map,
            Wallace::Specification::TemplateInstance
          ].include?(returns.class)
      deps = returns.dependencies
    else
      deps = []
    end

    # Calculate the dependencies for each parameter.
    return deps + accepts.all.inject([]) { |d, p| p.dependencies }

  end

end
