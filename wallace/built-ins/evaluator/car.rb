# encoding: utf-8
import('simple')
import('../chromosome/car')
import('../type/evaluator')
import('../fitness/simple')

define_type :car do
  extends(:evaluator)

  attribute :car_evaluator,    ruby(:proc)
  attribute :threads,           ruby(:integer)
  attribute :path_car,     ruby(:string)

  template lambda { |base, opts = {}, &fun|
             base.path_car(opts[:path])
             base.threads(opts[:threads] || 1)
             base.car_evaluator(fun)
           }
  composer do
    s = spec(:evaluator, :car).extend
    s.implement_method(:evaluate_car, &car_evaluator)
    s.constructor([], source: "super('#{path_car}', #{threads})")
    s
  end

end

define_specification :car do
  extends(:evaluator)

  ruby_file('tempfile')
  ruby_file('fileutils')
  ruby_file('pathname')

  attribute :path,    ruby(:string)
  attribute :threads, ruby(:integer)

  dependency s(:fitness, :simple)

  constructor [
                  p(:path,    ruby(:string)),
                  p(:threads, ruby(:integer))
              ] do
    self.path = Pathname.new(path).cleanpath.to_s
    self.threads = threads

    # Construct the evolved robots sub-directory.
    FileUtils.mkdir_p("#{self.path}/robots/sample/evolved")
  end

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

  abstract_method :evaluate_car, returns: s(:fitness), accepts: [
                                    p(:rng,             s(:rng)),
                                    p(:population,      s(:population)),
                                    p(:statistics,      s(:statistics)),
                                    p(:representation,  s(:representation)),
                                    p(:chromosome,      s(:chromosome)),
                                    p(:car,            s(:chromosome, :car))
                                ]

  method :evaluate, returns: ruby(:integer), accepts: [
                      parameter(:rng,         s(:rng)),
                      parameter(:statistics,  s(:statistics)),
                      parameter(:population,  s(:population)),
                      parameter(:termination, s(:termination))
                  ] do

    population.demes.each do |deme|
      candidates = (deme.offspring + deme.contents).reject { |i| i.evaluated == true }
      evals = candidates.length

      if termination.criterion?('evaluations')
        remaining = [0, termination.evaluations.limit - statistics.evaluations].max
        if remaining < evals
          evals = remaining
          candidates = rng.sample(candidates, remaining)
        end
      end

      workload = calculate_workload(evals, threads)


      #FileUtils.rm_rf("#{Dir.home}/UGV/evolved/.", secure: true)

      viable = 0
      cars = Array.new(evals)
      threads.times do |i|
        Thread.new(i) { |i|
          for i in workload[i][0] ... workload[i][1]
            begin
              cars[i] = build_car(candidates[i].species.representation.derive(candidates[i].genotype))
              viable += 1
            rescue
              cars[i] = nil
            end
          end
        }.join
      end


      # Evaluate the fitness of each of the tanks.
      threads.times do |i|
        Thread.new(i) { |i|
          for i in workload[i][0] ... workload[i][1]
            candidates[i].fitness = evaluate_car(
                rng,
                population,
                statistics,
                candidates[i].species.representation,
                candidates[i].genotype,
                cars[i]
            )
            candidates[i].evaluated = true
          end
        }.join
      end

      statistics.evaluations += evals

    end

  end

  method :build_car, returns: s(:chromosome, :car), accepts: [
                       parameter(:body, ruby(:string))
                   ] do
    Chromosome::Car.new(self.path, body)
  end

  # Returns the results of a battle between -----
  method :battle, returns: ruby(:*), accepts: [parameter(:body, ruby(:string))] do

    x = "#{body}"
    x.slice!("#<Chromosome::Car:")
    x.slice!(">")

    print "\n"
    print ("Evaluating candidate: #{x}")
    print "\n"

    cmd = "java -classpath ~/Project/UGV/bin dominant.SimulationWithEvolvedCars #{x}"
    
    system(cmd)

    forceOffLine = Array.new
    parked = Array.new
    crossedLine = Array.new
    collideCar = Array.new

    array = Array.new

    # if !(File.file?("FitnessLog.txt"))
    #   return 0
    # end

    File.open( "FitnessLog.txt" ).each do |line|

      array = line.split(' ')



      number1 = array[2].split(',')
      number1[0].slice!("car1Location:Double2D[")
      number1[1].slice!("]")
      number2 = array[3].split(',')
      number2[0].slice!("car2Location:Double2D[")
      number2[1].slice!("]")
      n1 = number1[0].to_i - number2[0].to_i
      n2 = number1[1].to_i - number2[1].to_i

      v1 = [n1,n2]


      if array[0] == "forcedOffRoad:"
        if forceOffLine.empty?
            forceOffLine.push(v1)
        else
          flag = true
          forceOffLine.each do |z|
            if(z[0] - v1[0]).abs > -0.1
              if(z[1] - v1[1]).abs > -0.1
                flag = false
              end
            end
          end
          if flag
            forceOffLine.push(v1)
          end
        end
      elsif array[0] == "collisionWithParkedCar:"
        if parked.empty?
          parked.push(v1)
        else
          flag = true
          parked.each do |z|
            if(z[0] - v1[0]).abs > -0.1
              if(z[1] - v1[1]).abs > -0.1
                flag = false
              end
            end
          end
          if flag
            parked.push(v1)
          end
        end
      elsif array[0] == "crossedLine:"
        if crossedLine.empty?
          crossedLine.push(v1)
        else
          flag = true
          crossedLine.each do |z|
            if(z[0] - v1[0]).abs > -0.1
              if(z[1] - v1[1]).abs > -0.1
                flag = false
              end
            end
          end
          if flag
            crossedLine.push(v1)
          end
        end
      elsif array[0] == "collisionWithOtherCar:"
        if collideCar.empty?
          collideCar.push(v1)
        else
          flag = true
          collideCar.each do |z|
            if(z[0] - v1[0]).abs > -0.1
              if(z[1] - v1[1]).abs > -0.1
                flag = false
              end
            end
          end
          if flag
            collideCar.push(v1)
          end
        end

      end
      array = Array.new
    end

    totalFitness = forceOffLine.count + parked.count + crossedLine.count + collideCar.count

    FileUtils.cp("FitnessLog.txt","UGV/evolved/#{x}/fitnessScore#{totalFitness}.txt")
    FileUtils.rm_rf("FitnessLog.txt", secure: true)

    return totalFitness

  end


end
