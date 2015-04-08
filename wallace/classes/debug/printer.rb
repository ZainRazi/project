# encoding: utf-8

class Wallace::Printer

  def self.print(s)
    puts compile(s)
  end

  def self.compile(s)
    header = "class #{s.class_name}"
    header << " < #{s.parent.class_name}" unless s.base?

    # Attributes
    body = ""
    body << "\n" unless s.attributes.empty?
    s.attributes.each do |n, a|
      body << "attribute :#{n}\n"
    end

    # Constructor
    unless s.get_constructor.nil?
      body << "\n#{compile_method(s.get_constructor)}"
    end

    # Methods
    body << "\n" unless s.methods.empty?
    s.methods.each do |n, m|
      body << compile_method(m)
    end

    body << "\n"
    body = body.lines.map(&:indent).join

    return "#{header}\n#{body}\nend"
  end

  def self.compile_attribute(a)
    "attr_accessor #{a.name}"
  end

  def self.compile_method(m)
    c = "def #{m.name.to_s}"
    c += (m.accepts.required.map { |p|
      p.name.to_s
    } + m.accepts.optional.map { |p|
      "#{p.name.to_s} = #{p.default.to_s}"
    }).join(', ').prepend('(').concat(")\n")
    c += m.source.indent
    c += "\nend"
    return c
  end

end
