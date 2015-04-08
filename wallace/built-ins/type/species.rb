# encoding: utf-8
import('bootstrap')
import('individual')
import('rng')
import('representation')

refine_specification :species do

  template_parameter  :individual,      s(:individual)
  template_parameter  :representation,  s(:representation)

  # The representation used by members of this species.
  attribute :representation, T[:representation]

  # Generates a new individual belonging to this species at pseudo-random.
  method :generate, returns: T[:individual], accepts: [
    p(:rng, :rng)
  ] do
    template(:individual).new(self, representation.generate(rng))
  end

end

refine_type :species do

  # The representation used by members of this species.
  attribute :representation, i(:representation)

  composer do |s_individual|

    # Compose the representation for this species and retrieve the chromosome
    # specification.
    si_representation = representation.compose
    s_representation = si_representation.specification
    s_chromosome = s_representation.ancestors[-1].parameters[:chromosome]

    # Put together the species specification, and inject the representation
    # initialisation into the constructor.
    s = spec(:species).extend(
      individual: s_individual.instantiate(chromosome: s_chromosome),
      representation: s_representation
    )
    s.constructor([], source: "self.representation = #{si_representation}")
    s

  end

end
