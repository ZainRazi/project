# encoding: utf-8
import('bootstrap')
import('species')

define_type :deme do
  attribute :species, t(:species)
  composer do |s_individual|
    s = spec(:deme).instantiate(
      individual: s_individual,
      species: species.compose(s_individual))
  end
end

define_specification :deme do

  # The species to which members of this deme belong.
  template_parameter :species, s(:species)

  # A list of the members of this deme.
  attribute :contents,  list(s(:individual))

  # A list of the current offspring for this deme.
  attribute :offspring, list(s(:individual))

  # The number of offspring which this deme produces at each generation.
  attribute :num_offspring, ruby(:integer)
  
  # The number of individuals which this deme can hold.
  attribute :capacity,  ruby(:integer)

  # The species of individuals belonging to this deme.t
  attribute :species,   T[:species]

  # Constructs a new deme.
  #
  # ==== Parameters
  # [+capacity+]  The number of individuals which this deme can hold.
  constructor [
    parameter(:capacity, ruby(:integer)),
    parameter(:num_offspring, ruby(:integer)) # code("capacity")
  ] do
    self.capacity = capacity
    self.num_offspring = num_offspring
    self.species = template(:species).new
    self.contents = []
    self.offspring = []
  end

  # Generates the contents of this deme at pseudo-random.
  method :generate, accepts: [
    parameter(:rng, s(:rng))
  ] do
    self.contents = Array.new(capacity) { species.generate(rng) }
  end

  # Removes all individuals from this deme.
  method :clear do
    self.contents.clear
  end

  # Finds and returns the individual with the best candidate solution
  # from this deme.
  method :best, returns: s(:individual) do
    best = nil
    contents.each do |ind|
      begin
        best = ind if ind.evaluated && ind > best
      rescue
        puts
        puts ind.inspect
        exit
      end
    end
    return best
  end

  # Returns the individual with the highest fitness within this deme.
  # method :max, returns: s(:individual) do
  #   max = nil
  #   self.contents.each do |ind|
  #     max = ind if ind.evaluated && (max.nil? || ind > max)
  #   end
  #   return max
  # end

  # Returns the individual with the lowest fitness within this deme.
  # method :min, returns: s(:individual) do
  #   min = nil
  #   self.contents.each do |ind|
  #     puts "Min(#{ind.fitness})"
  #     min = ind if ind.evaluated && (min.nil? || ind < min)
  #   end
  #   return min
  # end

  method :size, returns: ruby(:integer) do
    self.contents.size
  end

end
