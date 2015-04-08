# encoding: utf-8
import('bootstrap')
import('species')
import('chromosome')
import('fitness')

refine_specification :individual do

  uses_module :Comparable

  # The chromosome used by the genotype of this individual.
  template_parameter :chromosome, s(:chromosome)

  # The species that this individual belongs to.
  attribute :species,   s(:species)

  # The fitness of this individual.
  attribute :fitness,   s(:fitness)

  # A flag indicating whether this individual has been evaluated yet or not.
  attribute :evaluated, ruby(:boolean)

  # The genotype of this individual.
  attribute :genotype,  T[:chromosome]

  # Constructs a new individual.
  constructor [
    parameter(:species,   s(:species)),
    parameter(:genotype,  T[:chromosome])
  ] do
    self.species = species
    self.genotype = genotype
    self.evaluated = false
    self.fitness = nil
  end

  # Compares the fitness of this individual against another.
  method :<=>, returns: ruby(:integer), accepts: [
    parameter(:other, s(:individual))
  ] do
    return 1 if other.nil?
    #return 1 if other.evaluated
    begin
      return fitness <=> other.fitness
    rescue Exception
      puts "hello"
      exit
    end
  end

  # Produces a deep clone of this individual.
  method :clone, returns: ruby(:*) do
    Individual[template(:chromosome)].new(species, genotype.clone)
  end

  # Returns a string description of this individual.
  method :describe, returns: ruby(:string) do
    return species.representation.describe(genotype)
  end

end

refine_type :individual do
  composer { s(:individual) }
  #composer do |s_chromosome|
  #  #s(:individual).extend(chromosome: s_chromosome)
  #end
end
