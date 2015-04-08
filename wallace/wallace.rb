# encoding: utf-8
require 'singleton'
require 'pathname'
require 'tmpdir'

class Wallace
  include Singleton

  # Load the necessary refinements.
  require_relative 'refinements/string'

  # Load the language types.
  require_relative 'classes/language/code'
  require_relative 'classes/language/list'
  require_relative 'classes/language/map'
  require_relative 'classes/language/ruby_type'
  require_relative 'classes/language/wildcard'
  require_relative 'classes/language/boolean'

  # Load the class files.
  require_relative 'classes/specification'
  require_relative 'classes/type'

  require_relative 'classes/compiler'

  # Load the type and specification namespaces.
  require_relative 'classes/namespace/type'
  require_relative 'classes/namespace/specification'

  # Debugging.
  require_relative 'classes/debug/printer'

  # Executes a given block within the Wallace environment.
  def self.execute(&blk)
    instance.execute(&blk)
  end

  # Executes a given block within the Wallace environment.
  def execute(&blk)
    instance_eval(&blk)
  end

  def self.import(path)
    instance.import(path)
  end

  # Loads and executes a given script within the Wallace environment, as long
  # as it hasn't been imported already.
  #
  # ==== Parameters
  # [+path+]  The path to the script file which should be loaded.
  #
  # ==== Returns
  # True if the file is successfully imported, false if it has already been
  # imported.
  #
  # ==== Exceptions
  # [+Exception+] If no Ruby file exists at the given path.
  def import(path)

    # Add the ".rb" extension to the end of the file if it's missing and
    # record the relative path.
    path << '.rb' unless path.end_with?('.rb')

    # Calculate the absolute file location.
    path = File.join(File.dirname(caller[0]), path) unless Pathname.new(path).absolute?
    path = Pathname.new(path).realpath(path)
    path_s = path.to_s

    # Check that the file exists.
    if !path.exist?
      fail("Failed to find file: #{path_s}")
    end

    # Don't import the file if it has already been imported.
    return false if @loaded_files.include?(path_s)

    # Load the file contents, mark the file as imported, and execute it within the
    # Wallace environment.
    @loaded_files << path_s
    instance_eval(path.read, path.to_s, 0)
    return true

  end

  # Constructs the Wallace environment.
  def initialize

    # Register the Ruby classes.
    register ruby(:float, Float)
    register ruby(:integer, Integer)
    register ruby(:int, Integer)
    register ruby(:string, String)
    register ruby(:str, String)
    register ruby(:bool, Boolean)
    register ruby(:boolean, Boolean)
    register ruby(:symbol, Symbol)
    register ruby(:proc, Proc)
    register ruby(:block, Proc)
    register ruby(:lambda, Proc)
    register ruby(:*, Wildcard)
    register ruby(:random, Random)
    
    # Load all the built-ins.
    @loaded_files = []
    Dir.glob("#{File.dirname(__FILE__)}/built-ins/**/").each do |d|
      Dir.glob("#{d}/*.rb").each { |f| import(f) }
    end

  end

  # Attempts to register a given entity with the Wallace environment.
  def register(entity)
    if entity.is_a?(Type)
      return Namespace::Type.register(entity)
    elsif entity.is_a?(Specification)
      return Namespace::Specification.register(entity)
    elsif entity.is_a?(RubyType)
      return RubyType::Register.store(entity)
    else
      fail("Failed to register entity: unrecognised class.")
    end
  end

  # Registers a run mode with the Wallace kernel.
  def register_run_mode(name, template)

  end

  # ----------------------------------------------------------------
  # Convenience methods
  # ----------------------------------------------------------------
  def specification(*args, &blk)
    Specification.new(*args, &blk)
  end
  alias_method :spec, :specification
  alias_method :s, :specification

  def parameter(*args, &blk)
    Specification::Parameter.new(*args, &blk)
  end
  alias_method :param, :parameter
  alias_method :p, :parameter

  def type(*args, &blk)
    Type.new(*args, &blk)
  end
  alias_method :t, :type

  def ruby(*args, &blk)
    RubyType.new(*args, &blk)
  end

  # Defines a new specification.
  def define_specification(*args, &blk)
    register(specification(*args, &blk))
  end
  alias_method :def_specification, :define_specification
  alias_method :def_spec, :define_specification
  alias_method :def_s, :define_specification
  alias_method :define_spec, :define_specification
  alias_method :define_s, :define_specification

  # Defines a new type.
  def define_type(*args, &blk)
    register(type(*args, &blk))
  end
  alias_method :def_type, :define_type
  alias_method :def_t, :define_type
  alias_method :define_t, :define_type

  # Refines an existing specification.
  def refine_specification(spec, subspec = nil, &blk)
    Specification[spec, subspec].refine(&blk)
  end
  alias_method :ref_specification, :refine_specification
  alias_method :ref_spec, :refine_specification
  alias_method :ref_s, :refine_specification
  alias_method :refine_spec, :refine_specification
  alias_method :refine_s, :refine_specification

  # Refines an existing type.
  def refine_type(type, subtype = nil, &blk)
    Type[type, subtype].refine(&blk)
  end
  alias_method :ref_type, :refine_type
  alias_method :ref_t, :refine_type
  alias_method :refine_t, :refine_type

  # Compiles a given type and all its (recursive) dependencies in Ruby
  # to a target directory.
  def compile(dest, type, subtype = nil)
    t = Type[type, subtype]
    Compiler.new.compile(dest, t)
    puts "Compiled type: #{t.name}"
  end

  # Compiles a given setup to a temporary directory and executes it once.
  #
  # ==== Parameters
  # [+setup+] The name of the setup that
  # [+opts+]  A hash of keyword options to this method.
  #
  # ==== Options
  # [+mode+]  The platform to run the code on.
  def run(setup, opts = {})
    mode = opts[:mode] || :ruby
    Dir.mktmpdir("wallace_setup_") do |dir|
      compile(dir, :setup, setup)

      # For now...
      system("#{mode} '#{dir}/run.rb'")
    end
  end

  # ----------------------------------------------------------------
  # Debugging methods
  # ----------------------------------------------------------------
  def print(specification)
    Wallace::Printer.print(specification)
  end

end
