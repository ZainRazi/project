# encoding: utf-8
import('../type/representation')
import('grammar_derivation/literal')
import('grammar_derivation/symbol')
import('../chromosome/vector')

define_type :grammar_derivation do
  extends(:representation)
  composer { s(:representation, :grammar_derivation) }
end

define_specification :grammar_derivation do
  extends(:representation,
    chromosome: s(:chromosome, :vector).instantiate(of: ruby(:int)))

  # Dependencies.
  dependency s(:grammar_literal)
  dependency s(:grammar_symbol)

  attribute :length,    ruby(:integer)
  attribute :root,      ruby(:string)
  attribute :max_wraps, ruby(:integer)

  # For now...
  attribute :rules,     map(ruby(:string), list(list(ruby(:*))))

  # Constructs a new grammar derivation representation.
  constructor [
    parameter(:length,    ruby(:integer)),
    parameter(:root,      ruby(:string)),
    parameter(:max_wraps, ruby(:integer)),
    parameter(:rules,     map(ruby(:string), list(ruby(:string)))),
  ] do
    
    # Tokenize each of the grammar rule entries.
    rules.each_pair do |symbol, entries|

      # This would break CPP compatibility!
      rules[symbol] = entries.map { |e|
        tokens = []
        until e.empty?
          left, tag, right = e.partition(/<[^\<\>]+>/)
          tokens << GrammarLiteral.new(left) unless left.empty?
          tokens << GrammarSymbol.new(tag[1...-1]) unless tag.empty?
          e = right
        end
        tokens
      }

    end

    # Store the tokenized rules, along with all other attributes for
    # this representation.
    self.length = length
    self.rules = rules
    self.root = root
    self.max_wraps = max_wraps

  end

  # Returns a description of a given chromosome as a string.
  method :describe, returns: ruby(:string), accepts: [
    parameter(:chromosome, self)
  ] do
    begin
      return derive(chromosome)
    rescue
      return ''
    end
  end

  # Samples a gene value for this representation.
  method :sample_gene, returns: ruby(:integer), accepts: [
    parameter(:rng, s(:rng))
  ] do
    rng.integer
  end

  # Generates a new int vector at pseudo-random.
  implement_method :generate do
    c = Chromosome::Vector[Integer].new(length)
    for i in 0 ... length
      c.set(i, rng.integer)
    end
    return c
  end

  method :compile, returns: ruby(:proc), accepts: [
    parameter(:arguments,   list(ruby(:string))),
    parameter(:chromosome,  s(:chromosome, :vector).instantiate(of: ruby(:integer)))
  ] do
    begin
      f = derive(chromosome)
      f = "lambda { |#{arguments.join(',')}| #{f} }"
      return eval(f)
    rescue
      return nil
    end
  end

  method :derive, returns: ruby(:string), accepts: [
    parameter(:from, s(:chromosome, :vector).instantiate(of: ruby(:integer)))
  ] do

    # Construct a string to hold the derivation.
    derivation = ""

    # Keep processing the sequence of tokens until none remain.
    #
    # * Literal tokens are appended to the end of the derivation.
    # * Symbol tokens are converted into a given symbol derivation,
    #   using the next codon as the index if there is more than a single
    #   choice.
    # * When there are no codons left to consume then we either:
    #   a) Add a new codon to the sequence (until the limit is reached).
    #      [DISABLED]
    #   b) Reset the codon index to zero, wrapping the sequence round.
    queue = [GrammarSymbol.new(self.root)]
    codon_index = 0
    sequence_length = from.length
    num_wraps = 0

    until queue.empty?
      token = queue.shift

      if token.literal?
        derivation << token.value
      else
        options = rules[token.value]
        if options.length == 1
          queue = options[0] + queue
        else

          # Check if there are no remaining codons in the sequence.
          if codon_index >= sequence_length
            fail("Genotype exhausted.") if num_wraps == max_wraps
            #fail(Wallace::Errors::GenotypeExhaustedError)
            codon_index = 0
            num_wraps += 1
          end

          queue = options[from[codon_index] % options.length] + queue
          codon_index += 1
        end
      end

    end

    # Return the produced string.
    return derivation

  end

end
