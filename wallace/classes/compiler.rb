# encoding: utf-8
require 'fileutils'

class Wallace::Compiler

  # Calculates the destination file path for a given specification.
  def file_path(s, dest)
    file_name = s.name.to_s
    file_name.prepend("#{s.namespace.to_s}/") unless s.namespace == :type
    return "#{dest}/#{file_name}.rb"
  end

  # Calculates the relative file path from one specification to another.
  def relative_file_path(from, to)

    # Same namespace.
    if from.namespace == to.namespace
      return to.name.to_s

    # Type file.
    elsif to.namespace == :type
      return from.namespace == :type ? to.name.to_s : "../#{to.name}"

    # Subtype file.
    elsif from.namespace == :type
      return "#{to.namespace}/#{to.name}"

    else
      return "../#{to.namespace}/#{to.name}"
    end
    
  end

  # Compiles a given setup.
  def compile(dest, setup)

    # Clear out all the contents of the destination directory.
    FileUtils.rm_rf(Dir.glob("#{dest}/*"))

    # Copy each of the bootstrap files into the destination directory.
    compile_bootstrap(dest)

    dependencies = setup.compose.full_dependencies

    dependencies.each do |s|

      # Ensure the directory of the target file path exists before writing the
      # Ruby source code to the appropriate the file path.
      path = file_path(s, dest)
      FileUtils.mkdir_p(File.dirname(path))
      File.open(path, 'w') { |f| f.write(compile_specification(s, dest)) }

    end

    # Compile the run file.
    compile_run_file(dest, dependencies[0])

  end

  def compile_specification(s, dest)
    
    header = "# encoding: utf-8\n"

    # If this specification is a base specification, add a dependency on
    # the entity class.
    if s.base?
      header << "require_relative 'entity'\n"
    end

    # Inject any Ruby file dependencies.
    s.ruby_files.each do |f|
      header << "require '#{f}'\n"
    end

    # Inject the dependencies into this file via require statements.
    s.dependencies.keep_if { |d| !s.base? || d.namespace != s.name }.each do |d|
      header << "require_relative '#{relative_file_path(s, d)}'\n"
    end

    header << "\n"
    header << "class #{s.class_name}"

    # Calculate and inject the name of the parent class.
    header << (s.base? ? " < Entity" : " < #{s.parent.class_name}")

    # Compile any module inclusions for this specification.
    unless s.modules.empty?
      header << "\n"
      header << s.modules.map { |n| "  include #{n}\n" }.join
    end

    # Compile any template parameter definitions for this specification.
    if s.template?
      header << "\n"
      s.template_parameters.each do |n, v|
        header << "  template_parameter :#{n}\n"
      end
    end

    # If this specification is a base type, inject any dependencies
    # within its namespace here.
    s.dependencies.keep_if { |d| d.namespace == s.name }.each { |d|
      header << "  require_relative '#{relative_file_path(s, d)}'\n"
    } if s.base?

    # Compile the attributes of this specification.
    body = ""
    body << "\n" unless s.attributes.empty?
    s.attributes.each { |n, a| body << compile_attribute(a) }

    # Compile the constructor for this specification.
    unless s.get_constructor.nil?
      body << "\n#{compile_method(s, s.get_constructor)}\n"
    end

    # Compile the methods for this specification.
    body << "\n" unless s.methods.empty?
    s.methods.each do |n, m|
      body << "\n#{compile_method(s, m)}\n"
    end

    body << "\n"
    body = body.lines.map(&:indent).join

    return "#{header}\n#{body}\nend"
  end
  private :compile_specification

  def compile_attribute(a)
    "attr_accessor :#{a.name}\n"
  end
  private :compile_attribute

  def compile_method(s, m)

    # Inject the name of this class into all template parameter requests.
    src = m.source.gsub(/template\(:\w+\)/) { |req|
      "template(#{s.class_name}, #{req[9...-1]})"
    }.indent

    c = "def #{m.name.to_s}"
    c += (m.accepts.required.map { |p|
      p.name.to_s
    } + m.accepts.optional.map { |p|
      "#{p.name.to_s} = #{p.default.to_s}"
    }).join(', ').prepend('(').concat(")\n")
    c += src
    c += "\nend"
    return c
  end
  private :compile_method

  def compile_bootstrap(dest)
    FileUtils.cp("#{File.dirname(__FILE__)}/compiler/bootstrap/entity.rb",
      "#{dest}/entity.rb")
  end
  private :compile_bootstrap

  def compile_run_file(dest, setup_spec)
    source =  ["# encoding: utf-8"]
    source << "require_relative 'setup/#{setup_spec.name}'"
    source << ""
    source << "s = #{setup_spec.class_name}.new"
    source << "s.evolve"
    File.open("#{dest}/run.rb", 'w') { |f| f.write(source.join("\n")) }
  end
  private :compile_run_file

end
