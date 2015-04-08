# encoding: utf-8
import('simple')
import('../fitness/simple')

define_type :regression do
  extends(:evaluator, :simple, [threads: 1, data: []]) {}

  # The training data for this evaluator.
  attribute :data, ruby(:*)

  template lambda { |base, opts = {}, &fun|
    base.threads(opts[:threads] || 1)
    base.data(opts[:data] || [])
    base.candidate_evaluator(fun)
  }

  composer do
    s = sup()
    s.attribute(:data, ruby(:*))

    d = data.map { |row|
      row.map { |cell| eval(cell) }
    }.inspect

    s.constructor([], source: "
      super(#{threads})
      self.data=#{d}")
    s
  end
  
end

# The simple evaluator evaluates candidate individuals by passing each
# individual to a provided objective function in isolation.
define_specification :simple do
  extends(:evaluator)

  # Depends on the simple fitness.
  dependency s(:fitness, :simple)

  # The number of threads available for evaluation.
  # --- RNG SAFETY!
  attribute :threads, ruby(:integer)

  # Constructs a new simple evaluator.
  constructor [
    p(:threads, ruby(:integer))
  ] do
    self.threads = threads
  end

  # Computes and returns the fitness of a single candidate.
  abstract_method :evaluate_candidate, returns: s(:fitness), accepts: [
    p(:rng,             s(:rng)),
    p(:population,      s(:population)),
    p(:statistics,      s(:statistics)),
    p(:representation,  s(:representation)),
    p(:chromosome,      s(:chromosome)) # need more info!  
  ]

  method :calculate_workload, returns: list(list(ruby(:integer))), accepts: [
    p(:size,    ruby(:integer)),
    p(:threads, ruby(:integer))
  ] do
    workload = []
    div = size / threads
    mod = size % threads
    start = 0
    threads.times do |i|
      length = div + (mod > 0 && mod > i ? 1 : 0)
      workload << [start, start + length]
      start += length
    end
    return workload
  end

  implement_method :evaluate do

    # Evaluate the contents of each deme in series.
    population.demes.each do |deme|

      # Compute the list of individuals which require evaluation from the current
      # members of each deme, and their offspring. Avoid re-evaluating individuals
      # unless the problem being solved is dynamic.
      candidates = (deme.offspring + deme.contents).reject { |i| i.evaluated == true }

      # Calculate the number of individuals which will be evaluated.
      evals = candidates.length

      # If there is an evaluation limit on this algorithm, then further
      # restrict the list of candidates to the maximum number which may be
      # evaluated without breaching this limit. When restricting the number of
      # candidates, randomly chose which candidates will be evaluated and which
      # will not, to avoid any positional bias.

      # This code should be dynamically injected.
      if termination.criterion?('evaluations')
        remaining = [0, termination.evaluations.limit - statistics.evaluations].max
        if remaining < evals
          evals = remaining
          candidates = rng.sample(candidates, remaining)
        end
      end

      # Split the evaluations across each of the available threads.
      workload = calculate_workload(evals, threads)
      threads.times do |i|
        Thread.new(i) { |i|
          for i in workload[i][0] ... workload[i][1]
            candidates[i].fitness = evaluate_candidate(
              rng,
              population,
              statistics,
              candidates[i].species.representation,
              candidates[i].genotype
            )
            candidates[i].evaluated = true
          end
        }.join
      end

      # Update the statistics to reflect the recent evaluations for this deme.
      # -- Wouldn't be thread safe if we evaluated demes in parallel.
      statistics.evaluations += evals

    end

  end

end
