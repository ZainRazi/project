# encoding: utf-8
import('../type/representation')

define_type :robocode_controller do
  extends(:representation)

  # converts_many_from(:grammar_derivation, Code["

  #   # Compile each grammar derivation into a

  #   # Write each grammar derivation to a Java source file, before
  #   # compiling that source file into a class file.
  #   files = chromosomes.map { |c|
  #     Chromosome::JavaClass.new(NAME, contents)
  #   }

  #   # Compile all the source files together.
    

  # "])

  # Injects each of the conversion operations into the representation
  # and returns the complete specification.
  composer do
    s(:representation, :robocode_controller)
  end
end